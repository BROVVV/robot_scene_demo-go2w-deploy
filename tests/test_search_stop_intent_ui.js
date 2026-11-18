// Run the production reducer with a delayed worker and a reconnect snapshot.
// No browser, server, ROS, or physical executor is started.
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const assert = require('assert');
const source = fs.readFileSync(path.join(__dirname,
  '../app/manual_web_demo/static/search_ui.js'), 'utf8');
function extract(name) {
  const start = source.indexOf('  function ' + name + '(');
  assert(start >= 0);
  const end = source.indexOf('\n  }', start);
  return source.slice(start, end + 4);
}
const banners = [], timeline = [];
const ctx = {
  appState: {search: {session_id: 'one', status: 'RUNNING'}},
  els: {}, timelineCount: 0,
  showBanner: (...args) => banners.push(args),
  pushTimeline: event => timeline.push(event), renderAll() {},
};
vm.createContext(ctx);
vm.runInContext(['applySearchEvent', 'applyStateSnapshot', 'captureStopLifecycle',
  'restoreStopLifecycle'].map(extract).join('\n'), ctx);
function emit(type, payload = {}, timestamp = 100) {
  ctx.applySearchEvent({event_type: type, session_id: 'one', payload, timestamp});
}
emit('STOP_REQUESTED');
assert.equal(ctx.appState.search.status, 'STOPPING');
for (const type of ['SESSION_STARTED', 'PAUSED', 'RESUMED', 'TARGET_CONFIRMED',
  'SEARCH_STATE_CHANGED', 'STOP_REQUESTED']) {
  emit(type, {phase: 'OBSERVE'}, 101);
  assert.equal(ctx.appState.search.status, 'STOPPING', type);
  assert.equal(ctx.appState.search.phase, 'STOPPING', type);
  assert.equal(ctx.appState.search.stop_requested_at, 100, type);
}
assert.equal(banners.length, 0, 'late target must not announce completed search');
assert(timeline.some(event => event.event_type === 'TARGET_CONFIRMED'));
ctx.applyStateSnapshot({session_id: 'one', status: 'RUNNING', phase: 'VERIFY',
  observation: {bundle_id: 'late-poll-frame'}, timeline: []});
assert.equal(ctx.appState.search.status, 'STOPPING', 'old poll cannot undo request event');
assert.equal(ctx.appState.observation.bundle_id, 'late-poll-frame');
ctx.applyStateSnapshot({session_id: 'one', status: 'STOPPING', phase: 'STOPPING',
  stop_requested_at: 100, timeline: []});
emit('RESUMED');
assert.equal(ctx.appState.search.status, 'STOPPING', 'reconnect preserves stop intent');
emit('SEARCH_FINISHED', {result: 'OPERATOR_STOP'});
assert.equal(ctx.appState.search.status, 'OPERATOR_STOP');
assert.equal(ctx.appState.search.phase, 'OPERATOR_STOP');
emit('SESSION_STARTED');
assert.equal(ctx.appState.search.status, 'OPERATOR_STOP');
ctx.applyStateSnapshot({session_id: 'two', status: 'STARTING', timeline: []});
ctx.applySearchEvent({event_type: 'SESSION_STARTED', session_id: 'two', payload: {}});
assert.equal(ctx.appState.search.status, 'RUNNING', 'new session clears old stop intent');
// A synchronous terminal ACK can precede the request telemetry.
ctx.appState.search = {session_id: 'one', status: 'OPERATOR_STOP'};
emit('STOP_REQUESTED');
assert.equal(ctx.appState.search.status, 'OPERATOR_STOP');
console.log('stop-intent UI reducer: delayed events, reconnect, terminal ACK, restart passed');
