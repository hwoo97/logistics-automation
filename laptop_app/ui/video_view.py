"""영상 표시 — JPEG를 받아 캔버스에 맞춰 그리고, 위에 상태 문구를 겹쳐 쓴다. 상하·좌우 반전은 표시에만 적용."""
import io
import tkinter as tk

from PIL import Image, ImageOps, ImageTk


class VideoView(tk.Canvas):
    def __init__(self, master, width=480, height=360):
        super().__init__(master, width=width, height=height, bg="#202020", highlightthickness=0)
        self.w, self.h = width, height
        self._photo = None
        self.flip_v = False      # 상하 반전 (카메라를 뒤집어 단 경우)
        self.flip_h = False      # 좌우 반전
        self._image_id = self.create_image(0, 0, anchor="nw")
        self._text_id = self.create_text(8, height - 8, anchor="sw", fill="#00ff66",
                                         font=("Consolas", 11, "bold"), text="영상 없음")

    def show(self, jpeg: bytes) -> None:
        try:
            img = Image.open(io.BytesIO(jpeg)).convert("RGB")
        except OSError:
            return
        if self.flip_v:
            img = ImageOps.flip(img)
        if self.flip_h:
            img = ImageOps.mirror(img)
        img = img.resize((self.w, self.h), Image.BILINEAR)
        self._photo = ImageTk.PhotoImage(img)      # 참조를 잡아 두지 않으면 사라짐
        self.itemconfigure(self._image_id, image=self._photo)

    def set_overlay(self, text: str) -> None:
        self.itemconfigure(self._text_id, text=text)
