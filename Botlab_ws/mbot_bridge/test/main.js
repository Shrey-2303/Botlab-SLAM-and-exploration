window.addEventListener("DOMContentLoaded", () => {
  console.log("Hello!");
  const mbotIP = window.location.host.split(":")[0]  // Grab the IP from which this page was accessed.
  const mbot = new MBotAPI.MBot(mbotIP);
  mbot.drive(0, 0, 0);
  mbot.readOdometry((odom) => { console.log("Odom:", odom); });

  let sub = false;
  document.getElementById('subscribeButton').addEventListener('click', function () {
    if (!sub) {
      mbot.subscribe(config.ODOMETRY.channel, (odom) => { console.log("SUB:", odom); });
      sub = true;
    }
    else {
      mbot.unsubscribe(config.ODOMETRY.channel);
      sub = false;
    }
  });
});
