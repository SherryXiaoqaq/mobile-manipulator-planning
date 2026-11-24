/* Capture the offline replay with Playwright and encode it with FFmpeg. */
'use strict';

const { spawn, spawnSync } = require('node:child_process');
const { once } = require('node:events');
const { rename, rm } = require('node:fs/promises');
const path = require('node:path');
const { pathToFileURL } = require('node:url');
const { chromium } = require(process.env.TOPIC2_PLAYWRIGHT_MODULE || 'playwright');

const [sceneId, method = 'coupled'] = process.argv.slice(2);
const root = path.resolve(__dirname, '..');
const results = path.join(root, 'results');
const finalVideo = path.join(results, method === 'overlap' ? 'simulation_overlap.mp4' : 'simulation.mp4');
const tempVideo = path.join(results, `.simulation-${process.pid}.mp4`);
const ffmpeg = process.env.TOPIC2_FFMPEG || 'ffmpeg';
const fps = 24;
const playbackSpeed = 1.5;

function encoderArguments() {
    const probe = spawnSync(ffmpeg, ['-hide_banner', '-encoders'], { encoding: 'utf-8' });
    if (probe.error) throw probe.error;
    if (/\blibx264\b/.test(probe.stdout)) return ['-c:v', 'libx264', '-preset', 'fast', '-crf', '20'];
    if (/\bh264_mf\b/.test(probe.stdout)) return ['-c:v', 'h264_mf', '-b:v', '2200k'];
    throw new Error('FFmpeg requires the libx264 or h264_mf encoder.');
}

async function capture(page, duration) {
    const encoder = spawn(ffmpeg, [
        '-hide_banner', '-loglevel', 'error', '-y', '-f', 'image2pipe', '-vcodec', 'mjpeg',
        '-framerate', String(fps), '-i', 'pipe:0', ...encoderArguments(),
        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', tempVideo,
    ], { stdio: ['pipe', 'ignore', 'inherit'] });
    // Keep an early encoder exit from producing an unhandled pipe error.
    let pipeError;
    encoder.stdin.on('error', error => { pipeError = error; });
    const exit = once(encoder, 'close');
    try {
        const start = await page.screenshot({ type: 'jpeg', quality: 91 });
        const motionFrames = Math.ceil(duration * fps / playbackSpeed);
        const send = async image => {
            if (pipeError) throw pipeError;
            if (!encoder.stdin.write(image)) {
                await Promise.race([once(encoder.stdin, 'drain'), exit]);
                if (pipeError || encoder.exitCode !== null) throw pipeError || new Error('FFmpeg exited early.');
            }
        };
        for (let frame = 0; frame < fps; frame++) await send(start);
        for (let frame = 1; frame <= motionFrames; frame++) {
            await page.evaluate(t => window.replayExport.seek(t), frame * playbackSpeed / fps);
            await send(await page.screenshot({ type: 'jpeg', quality: 91 }));
        }
        const end = await page.screenshot({ type: 'jpeg', quality: 91 });
        for (let frame = 0; frame < 2 * fps; frame++) await send(end);
        encoder.stdin.end();
        const [code] = await exit;
        if (code !== 0) throw new Error(`FFmpeg exited with code ${code}.`);
        await rename(tempVideo, finalVideo);
    } catch (error) {
        encoder.stdin.destroy();
        encoder.kill();
        await exit.catch(() => {});
        throw error;
    }
}

async function main() {
    const options = process.env.TOPIC2_BROWSER_CHANNEL ? { channel: process.env.TOPIC2_BROWSER_CHANNEL } : {};
    const browser = await chromium.launch(options);
    try {
        const page = await browser.newPage({ viewport: { width: 1280, height: 720 }, deviceScaleFactor: 1 });
        const replay = pathToFileURL(path.join(results, 'demo.html'));
        replay.search = new URLSearchParams({ video: '1', speed: String(playbackSpeed), method }).toString();
        await page.goto(replay.href);
        await page.waitForFunction(() => !!window.replayExport);
        if (sceneId) await page.evaluate(([scene, key]) => window.replayExport.select(scene, key), [sceneId, method]);
        await page.evaluate(() => document.fonts.ready);
        const duration = await page.evaluate(() => window.replayExport.duration());
        if (duration <= 0) throw new Error('The selected experiment has no replayable trajectory.');
        const current = await page.evaluate(() => window.replayExport.selection());
        if (current.method === 'overlap') {
            for (const [phase, suffix] of [['extend_pick','extend'], ['retract_pick','retract']]) {
                const t = await page.evaluate(phase => window.replayExport.syncTime(phase), phase);
                if (t === null) continue;
                await page.evaluate(t => window.replayExport.seek(t), t);
                await page.locator('#map').screenshot({ path: path.join(results, `trajectory_overlap_${suffix}.png`) });
            }
        } else {
            const seekPickup = () => window.replayExport.seek(window.replayExport.pickupTime());
            await page.evaluate(seekPickup);
            await page.locator('#map').screenshot({ path: path.join(results, 'trajectory.png') });
            if (current.hasSequential && current.method !== 'sequential') {
                await page.evaluate(scene => window.replayExport.select(scene, 'sequential'), current.sceneId);
                await page.evaluate(seekPickup);
                await page.locator('#map').screenshot({ path: path.join(results, 'trajectory_sequential.png') });
                await page.evaluate(({sceneId, method}) => window.replayExport.select(sceneId, method), current);
            }
        }
        await page.evaluate(() => window.replayExport.seek(0));
        await capture(page, duration);
        console.log(`Video written to ${path.basename(finalVideo)}`);
    } finally {
        await browser.close();
        await rm(tempVideo, { force: true });
    }
}

main().catch(error => { console.error(error); process.exitCode = 1; });
