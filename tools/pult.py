"""Пульт Мини BD-1: окно с кнопками, шлёт команды прошивке по USB.
Запуск: pult.bat (или python pult.py COM9). Пока пульт открыт, монитор порта открыть нельзя, и наоборот."""
import sys, threading, time
import tkinter as tk
import serial
from serial.tools import list_ports


def find_port():
    if len(sys.argv) > 1:
        return sys.argv[1]
    for p in list_ports.comports():
        if any(k in (p.description or "") for k in ("CH340", "CP210", "USB-SERIAL", "USB Serial")):
            return p.device
    return None


class Pult:
    def __init__(self, root):
        self.root, self.ser, self.busy = root, None, False
        root.title("Мини BD-1")
        root.resizable(False, False)
        self.status = tk.StringVar(value="подключаюсь…")
        tk.Label(root, textvariable=self.status, width=34, anchor="w").grid(row=0, column=0, columnspan=2, padx=10, pady=(10, 6))
        buttons = [
            ("Встать (стойка)", lambda: self.run(["stand"])),
            ("Выпрямиться", lambda: self.run(["straight"])),
            ("Идти 8 циклов", lambda: self.walk(8)),
            ("Идти без остановки", lambda: self.walk(None)),
            ("Присесть 5 раз", self.squats),
            ("Стоп", lambda: self.send("stop")),
            ("Проверить связь", self.check),
        ]
        for i, (text, fn) in enumerate(buttons):
            tk.Button(root, text=text, width=16, height=2, command=fn).grid(row=1 + i // 2, column=i % 2, padx=6, pady=4)
        self.info = tk.StringVar(value="")
        tk.Label(root, textvariable=self.info, width=34, anchor="w", justify="left", wraplength=260).grid(row=6, column=0, columnspan=2, padx=10, pady=(0, 10))
        tk.Label(root, text="Темп шага, с на цикл:").grid(row=5, column=0, sticky="e", padx=6, pady=(6, 10))
        self.period = tk.Scale(root, from_=1.2, to=4.0, resolution=0.2, orient="horizontal", length=140,
                               command=lambda v: self.send(f"period {v}"))
        self.period.set(2.0)
        self.period.grid(row=5, column=1, padx=6, pady=(0, 10))
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(100, self.connect)

    def connect(self):
        port = find_port()
        if not port:
            self.status.set("ESP32 не найдена: воткни USB и перезапусти")
            return
        try:
            s = serial.Serial(); s.port = port; s.baudrate = 115200; s.timeout = 0.1
            s.dtr = False; s.rts = False
            s.open()
        except serial.SerialException:
            self.status.set(f"{port} занят: закрой монитор порта и перезапусти пульт")
            return
        # чистая перезагрузка в обычный режим: после резко закрытого монитора ESP32 может висеть
        # в режиме прошивки (лампочка PCA горит, а серво не слушаются)
        s.rts = True; time.sleep(0.1); s.rts = False
        self.ser = s
        self.status.set(f"{port}: перезагружаю ESP32…")
        def boot():
            self.wait_for("Готово", 5)
            self.status.set(f"подключено: {port}, ноги прямо")
        threading.Thread(target=boot, daemon=True).start()

    def send(self, cmd):
        if not self.ser:
            return
        try:
            self.ser.write((cmd + "\n").encode())
        except serial.SerialException:
            self.ser = None
            self.status.set("связь пропала: проверь USB и перезапусти")

    def wait_for(self, word, secs):
        t, buf = time.time(), b""
        while time.time() - t < secs and self.ser:
            buf += self.ser.read(256)
            if word.encode() in buf:
                break

    def background(self, fn, label):
        if self.busy or not self.ser:
            return
        self.busy = True
        self.status.set(label)
        def work():
            try:
                fn()
                self.status.set("готов")
            except serial.SerialException:
                self.ser = None
                self.status.set("связь пропала: проверь USB и перезапусти")
            finally:
                self.busy = False
        threading.Thread(target=work, daemon=True).start()

    def run(self, cmds):
        def fn():
            for c in cmds:
                self.ser.reset_input_buffer(); self.send(c); self.wait_for("готово", 3)
        self.background(fn, " → ".join(cmds))

    def walk(self, n):
        def fn():
            self.ser.reset_input_buffer(); self.send("stand"); self.wait_for("готово", 3)
            self.send(f"walk {n}" if n else "walk")
            if n:
                self.wait_for("стою", n * float(self.period.get()) + 5)
                self.send("straight"); self.wait_for("готово", 3)
            else:
                self.wait_for("стою", 3600)   # до кнопки «Стоп»
        self.background(fn, f"иду {n} циклов…" if n else "иду, «Стоп» — остановить")

    def check(self):
        def fn():
            self.ser.reset_input_buffer(); self.send("status")
            t, buf = time.time(), b""
            while time.time() - t < 1.5 and b"\n" not in buf.split(b"I2C", 1)[-1]:
                buf += self.ser.read(256)
            line = next((l for l in buf.decode("utf-8", "replace").splitlines() if l.startswith("I2C")), None)
            self.info.set(line or "робот не ответил: передёрни USB у ESP32 и перезапусти пульт")
        self.background(fn, "проверяю связь…")

    def squats(self):
        def fn():
            self.ser.reset_input_buffer(); self.send("straight"); self.wait_for("готово", 3)
            for _ in range(5):
                self.send("stand"); self.wait_for("готово", 3)
                self.send("straight"); self.wait_for("готово", 3)
        self.background(fn, "приседаю…")

    def close(self):
        if self.ser:
            self.ser.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    Pult(root)
    root.mainloop()
