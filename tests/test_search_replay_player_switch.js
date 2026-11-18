// Regression for R12 (reports/go2w_search_replay_review_20261021.md):
// A 有视频 → B 无视频 → 返回 A 后面板必须能重新拿到 src 并真正加载。
//
// 直接执行生产函数（不复制实现），DOM/video 用最小替身。
// Run: node tests/test_search_replay_player_switch.js
const fs = require('fs');
const vm = require('vm');
const path = require('path');
const assert = require('assert');

const project = path.resolve(__dirname, '..');
const source = fs.readFileSync(
  path.join(project, 'app/manual_web_demo/static/search_ui.js'), 'utf8',
);

function extract(name) {
  const start = source.indexOf('  function ' + name + '(');
  assert(start >= 0, 'missing function ' + name);
  const end = source.indexOf('\n  }', start);
  return source.slice(start, end + 4);
}

const attrs = new Map();
let loads = 0;
const video = {
  classList: {add() {}, remove() {}},
  pause() {},
  load() {loads += 1;},
  getAttribute(name) {return attrs.has(name) ? attrs.get(name) : null;},
  setAttribute(name, value) {attrs.set(name, value);},
  removeAttribute(name) {attrs.delete(name);},
  set src(value) {attrs.set('src', value);},
  get src() {return attrs.has('src') ? attrs.get('src') : '';},
};

const context = {els: {replayPanel: {classList: {remove() {}}},
                       replayVideo: video}, replaySessionId: null};
vm.createContext(context);
vm.runInContext(
  extract('showReplayPanel') + '\n' + extract('attachReplayVideo'), context,
);

const statusA = {
  urls: {video: '/api/search/history/search_A/replay/video'},
  recording_id: 'rec_A', speed: 10, size_bytes: 123, duration: 1,
};

context.showReplayPanel('search_A');
context.attachReplayVideo(statusA);
const firstSrc = video.src;
assert(firstSrc, 'A 首次应写入 src');

// B 还没有成品：只切面板，不调用 attachReplayVideo。
context.showReplayPanel('search_B');
assert.strictEqual(video.getAttribute('src'), null, '切到 B 时应清掉 src');
assert.strictEqual(video.getAttribute('data-src'), null,
  '切到 B 时 data-src 也必须清掉，否则切回 A 会拒绝重载');

const loadsBefore = loads;
context.showReplayPanel('search_A');
context.attachReplayVideo(statusA);
assert.strictEqual(video.src, firstSrc, '返回 A 后必须重新挂上同一个地址');
assert(loads > loadsBefore, '返回 A 后必须真的触发一次 load()');

// 同一会话重复渲染状态更新不应该反复重载。
const loadsAfter = loads;
context.attachReplayVideo(statusA);
assert.strictEqual(loads, loadsAfter, '地址没变时不应重复 load()');

console.log(JSON.stringify({
  sequence: ['A playable', 'B unavailable', 'A playable'],
  first_src: firstSrc, final_src: video.src, loads,
}));
