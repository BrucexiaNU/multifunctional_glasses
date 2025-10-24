<h1 align="center">🕶️ multifunctional_glasses — User Guidance</h1>

<h2>📦 System Overview</h2>

<h3>⚙️ Configuration: <b>2ch_IMU + 2ch_BCM</b></h3>
<h3>📊 Sample Rate: <b>IMU – 100 Hz</b> | <b>BCM – 40 kHz</b></h3>

<p>
In the <code>2bcm_2imu</code> folder:
<ul>
  <li>The <b>.ino</b> file is the <b>Arduino firmware</b> for <b>nRF1</b>.</li>
  <li>The <b>.py</b> file runs on the <b>PC</b> side for <b>data collection</b>.</li>
</ul>
</p>

<p align="center">
  <img src="./image/back_detail.png" alt="back_detail" width="400"/>
  <br>
  <i>▲ Back-side connection layout</i>
</p>

<hr>

<h2>🔌 How to Use</h2>

<ol>
  <li>Use a <b>Type-C cable</b> to connect the nRF device to your computer.</li>
  <li>Run the provided <code>.py</code> Python program to start data collection.</li>
  <li>Follow terminal prompts:
    <ul>
      <li>Press <b>Enter</b> to start recording.</li>
      <li>Press <b>Enter</b> again to stop recording.</li>
    </ul>
  </li>
  <li>The collected data will be automatically saved.</li>
</ol>

<hr>

<h2>🎧 Output Files</h2>

<ul>
  <li><b>BCM data</b> → Saved as:
    <ul>
      <li><code>dual-channel .wav</code> file (for playback)</li>
      <li><code>raw_data.txt</code> file (for analysis)</li>
    </ul>
  </li>

  <li><b>IMU data</b> → Saved as:
    <ul>
      <li><code>IMU.txt</code> file (raw numeric data)</li>
      <li>Sampling rate adjustable in <b>firmware (.ino)</b></li>
    </ul>
  </li>
</ul>

<hr>

<h2>📄 IMU Data Format</h2>

<p>
Each generated <code>IMU.txt</code> file contains a sequence of IMU samples recorded from two sensors (<b>IMU1</b> and <b>IMU2</b>).
<br><br>
Each line contains <b>18 values</b> separated by spaces, following this structure:
</p>

<pre>
[ax1 ay1 az1 gx1 gy1 gz1 mx1 my1 mz1  ax2 ay2 az2 gx2 gy2 gz2 mx2 my2 mz2]
</pre>

<p>
Within each 9-value group:
<ul>
  <li><b>1–3:</b> Accelerometer (<i>ax, ay, az</i>)</li>
  <li><b>4–6:</b> Gyroscope (<i>gx, gy, gz</i>)</li>
  <li><b>7–9:</b> Magnetometer (<i>mx, my, mz</i>)</li>
</ul>
</p>

<hr>

<p align="center">
  <b>🧠 Tip:</b> You can adjust IMU sampling rates or modify save formats directly in the firmware to suit your application.
</p>
