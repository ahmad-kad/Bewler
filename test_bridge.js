const ROSLIB = require('roslib');

// Test ROS Bridge connection stability
console.log('Testing ROS Bridge connection...');

const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

let connectionCount = 0;
let errorCount = 0;
let closeCount = 0;
const startTime = Date.now();

ros.on('connection', () => {
  connectionCount++;
  const elapsed = Date.now() - startTime;
  console.log(`[${elapsed}ms] Connected! (Total connections: ${connectionCount})`);

  // Subscribe to a topic to verify data flow
  const listener = new ROSLIB.Topic({
    ros: ros,
    name: '/chatter',
    messageType: 'std_msgs/String'
  });

  listener.subscribe(function(message) {
    const elapsed = Date.now() - startTime;
    console.log(`[${elapsed}ms] Received: ${message.data}`);
  });
});

ros.on('error', function(error) {
  errorCount++;
  const elapsed = Date.now() - startTime;
  console.log(`[${elapsed}ms] Error: ${error} (Total errors: ${errorCount})`);
});

ros.on('close', function() {
  closeCount++;
  const elapsed = Date.now() - startTime;
  console.log(`[${elapsed}ms] Connection closed (Total closes: ${closeCount})`);
});

// Run test for 30 seconds
setTimeout(() => {
  console.log('\n=== Test Results ===');
  console.log(`Total connections: ${connectionCount}`);
  console.log(`Total errors: ${errorCount}`);
  console.log(`Total closes: ${closeCount}`);
  console.log('Expected: 1 connection, 0 errors, 0 closes (stable connection)');

  if (connectionCount === 1 && errorCount === 0 && closeCount === 0) {
    console.log('✅ PASS: Stable connection maintained');
  } else {
    console.log('❌ FAIL: Connection instability detected');
  }

  ros.close();
  process.exit(connectionCount === 1 && errorCount === 0 && closeCount === 0 ? 0 : 1);
}, 30000);