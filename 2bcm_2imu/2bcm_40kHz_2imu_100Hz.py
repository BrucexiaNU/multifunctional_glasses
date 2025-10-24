import os
import sys
import threading
from threading import Thread, Lock, Event
import serial
import numpy as np
import soundfile as sf
from serial.tools import list_ports
import time

AUDIO_MAGIC = b'\x00\x11\x22\x33'
IMU_MAGIC   = b'IMU2'
AUDIO_SR    = 40000 #audio sample rate
IMU_SR      = 100   #imu sample rate
AUDIO_FRAME_SAMPLES = 1600  
BAUD = 2_000_000

#connect to nrf
def get_nrf():
    ports = list_ports.comports()
    for port in ports:
        if sys.platform == 'win32':
            match = port.pid == 32837
        else:
            match = 'nRF52840' in port.description
        if match:
            print(f'Detected nRF on {port.device}')
            return port.device
    raise RuntimeError('Cannot find nRF')


class MultiFrameParser:
    def __init__(self):
        self.buf = bytearray()

    def add_data(self, data: bytes):
        self.buf.extend(data)

    def _find_next_marker(self):
        ia = self.buf.find(AUDIO_MAGIC)
        ii = self.buf.find(IMU_MAGIC)
        cand = []
        if ia != -1: cand.append((ia, 'audio'))
        if ii != -1: cand.append((ii, 'imu'))
        if not cand: return -1, None
        cand.sort(key=lambda x: x[0])
        return cand[0]

    def parse(self):
        out = []
        while True:
            if len(self.buf) < 4:
                break

            if not (self.buf.startswith(AUDIO_MAGIC) or self.buf.startswith(IMU_MAGIC)):
                idx, _ = self._find_next_marker()
                if idx == -1:
                    if len(self.buf) > 65536:
                        self.buf.clear()
                    break
                if idx > 0:
                    del self.buf[:idx]
                if len(self.buf) < 4:
                    break

            # ---- audio frame ----
            if self.buf.startswith(AUDIO_MAGIC):
                if len(self.buf) < 6: break
                n = int.from_bytes(self.buf[4:6], 'big')  # int16 
                total = 6 + n * 2
                if len(self.buf) < total: break

                mv = memoryview(self.buf)
                try:
                    arr = np.frombuffer(mv[6:total], dtype='<i2').copy()
                finally:
                    try:
                        mv.release()
                    except AttributeError:
                        pass
                    del mv

                del self.buf[:total]
                out.append({'type': 'audio', 'data': arr})
                continue

            # ---- IMU frame ----
            if self.buf.startswith(IMU_MAGIC):
                if len(self.buf) < 6: break
                count = int.from_bytes(self.buf[4:6], 'big')
                bytes_per_group = 18 * 2  # 18 int16
                total = 6 + count * bytes_per_group
                if len(self.buf) < total: break

                mv = memoryview(self.buf)
                try:
                    arr = np.frombuffer(mv[6:total], dtype='<i2').reshape((-1, 18)).copy()
                finally:
                    try:
                        mv.release()
                    except AttributeError:
                        pass
                    del mv

                del self.buf[:total]
                out.append({'type': 'imu', 'data': arr, 'count': count})
                continue

            del self.buf[:1]

        return out


class SerialAudioIMURecorder(Thread):
    """Recording control:
    - Receive the first IMU packet: Start recording (discard the IMU packet)
    - Receive a stop request: Continue until the next IMU packet is received, then stop
    """
    def __init__(self, port):
        super().__init__()
        self.port = port
        self.serial = serial.Serial(self.port, BAUD)
        self.serial.reset_input_buffer()
        self.parser = MultiFrameParser()
        self.lock = Lock()

        # status
        self.recording = False
        self.wait_first_imu = True
        self.pending_stop = False
        self.stop_latched = Event()
        self.stopped = False

        # buffer
        self.audio = {'mic': [], 'bcm': []}
        self.imu_rows = []

        print(f'Opening {self.port} @ {BAUD}...')

    def run(self):
        try:
            while not self.stopped:
                to_read = self.serial.in_waiting or 8192
                self.parser.add_data(self.serial.read(to_read))
                frames = self.parser.parse()
                if frames:
                    self._handle_frames(frames)
        finally:
            self.close()

    def request_stop(self):
        with self.lock:
            self.pending_stop = True

    def _handle_frames(self, frames):
        with self.lock:
            for f in frames:
                if f['type'] == 'imu':
                    if self.wait_first_imu:
                        self.wait_first_imu = False
                        self.recording = True
                        continue

                    if self.pending_stop:
                        self.imu_rows.append(f['data'])
                        self.recording = False
                        self.pending_stop = False
                        self.stop_latched.set()
                        self.stopped = True
                        return

                    self.imu_rows.append(f['data'])

                elif f['type'] == 'audio':
                    if not self.recording:
                        continue
                    pkt = f['data']
                    mic = pkt[::2]
                    bcm = pkt[1::2]
                    self.audio['mic'].append(mic)
                    self.audio['bcm'].append(bcm)

    #  align_bcm and IMU
    def align_bcm_imu(self, mic, bcm, imu_rows):
        if imu_rows:
            imu_all = np.vstack(imu_rows)
            imu_samples = imu_all.shape[0]
        else:
            imu_samples = 0

        if imu_samples == 0 or len(bcm) == 0:
            print('[Align] Cannot align（IMU or BCM is empty）')
            return mic, bcm, imu_rows

        target_bcm_len = int(imu_samples * (AUDIO_SR / IMU_SR))
        current_len = len(bcm)
        diff = target_bcm_len - current_len

        if diff == 0:
            print('[Align] BCM align with IMU')
        elif diff > 0:
            pad = np.zeros(diff, dtype=bcm.dtype)
            bcm = np.concatenate([bcm, pad])
            mic = np.concatenate([mic, np.zeros(diff, dtype=mic.dtype)])
            # print(f'[Align] BCM lack data，add number of {diff} samples')
        else:
            # longer BCM data
            bcm = bcm[:target_bcm_len]
            mic = mic[:target_bcm_len]
            print(f'[Align] BCM to long, drop {-diff} samples')

        return mic, bcm, imu_rows

    def savefile(self, filename):
        with self.lock:
            self._do_savefile(filename)

    def _do_savefile(self, filename):
        output_dir = "B:/251253/imu_shoes/bcm/2025814/test"
        os.makedirs(output_dir, exist_ok=True)

        mic = np.concatenate(self.audio['mic']) if self.audio['mic'] else np.empty((0,), dtype=np.int16)
        bcm = np.concatenate(self.audio['bcm']) if self.audio['bcm'] else np.empty((0,), dtype=np.int16)

        # ======= align BCM & IMU =======
        mic, bcm, self.imu_rows = self.align_bcm_imu(mic, bcm, self.imu_rows)

        # ======= save it as 2 channel audio =======
        min_len = min(len(mic), len(bcm))
        stereo_data = np.stack([mic[:min_len], bcm[:min_len]], axis=1)  # (N, 2)

        bcm_wav = os.path.join(output_dir, f'{filename}_bcm.wav')
        bcm_txt = os.path.join(output_dir, f'{filename}_bcm.txt')

        if stereo_data.size > 0:
            np.savetxt(bcm_txt, stereo_data, fmt='%d')
            sf.write(bcm_wav, stereo_data, AUDIO_SR, format='wav')

        # save IMU TXT：（9 for IMU1 + 9 for IMU2）
        imu_txt = os.path.join(output_dir, f'{filename}_imu.txt')
        if self.imu_rows:
            imu_all = np.vstack(self.imu_rows)
            np.savetxt(imu_txt, imu_all, fmt='%d')
            print(f'[IMU] {imu_all.shape[0]} samples saved -> {imu_txt}')
        else:
            open(imu_txt, 'w').close()
            print('[IMU] 0 samples')

        print(f'[BCM] saved {stereo_data.shape[0]} samples ({stereo_data.shape[0]/AUDIO_SR:.2f}s)')
        print(f'[Files] {bcm_wav}')
        print(f'        {bcm_txt}')

        self._clear_internal()

    def clear(self):
        with self.lock:
            self._clear_internal()
            self.recording = False
            self.wait_first_imu = True
            self.pending_stop = False
            self.stop_latched.clear()

    def _clear_internal(self):
        self.audio = {'mic': [], 'bcm': []}
        self.imu_rows = []

    def close(self):
        try:
            print(f'Closing {self.port}...')
            self.serial.close()
        except Exception:
            pass


if __name__ == '__main__':
    port = get_nrf()
    rec = SerialAudioIMURecorder(port)

    fname = input('Enter filename: ')
    time.sleep(0.5)
    rec.start()

    print('\n=== instuction ===')
    print('After startup, wait for the first IMU packet to arrive: start recording from the IMU boundary (the packet is discarded), and then record BCM and IMU simultaneously')
    print('Press Enter to stop: The program will continue until it receives the next IMU packet, count the IMU packet, and then stop and save.\n')

    try:
        while True:
            choice = input("Enter(stop and save) / r(record again) / Ctrl+C(quit): ").strip().lower()
            if choice == 'r':
                rec.clear()
                print('[*] Cleared buffers. Waiting for next IMU to start recording...')
                continue
            else:
                rec.request_stop()
                print('[*] Stop requested. Waiting for next IMU boundary (this one will be included)...')
                rec.stop_latched.wait()
                rec.savefile(fname)
                break
    except KeyboardInterrupt:
        pass

    rec.stopped = True
    rec.join()
