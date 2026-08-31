"""Deterministic bilingual vocabulary used when an LLM is unavailable."""

from __future__ import annotations


COMMON_OBJECT_VOCAB: dict[str, list[str]] = {
    "手机": ["phone", "cell phone", "mobile phone", "smartphone", "electronic device"],
    "钥匙": ["key", "keys", "keychain", "metal key"],
    "遥控器": ["remote control", "remote", "controller"],
    "打印机": ["printer", "copier", "multifunction printer", "office machine", "paper tray"],
    "垃圾桶": ["trash can", "trash bin", "waste bin", "garbage bin"],
    "消防器材": ["fire extinguisher", "fire equipment", "red cylinder", "emergency equipment"],
    "灭火器": ["fire extinguisher", "portable extinguisher", "fire suppression unit"],
    "柜门": ["cabinet door", "door panel", "cabinet", "handle"],
    "门": ["door", "doorway", "entrance"],
    "柜子": ["cabinet", "closet", "wardrobe", "cupboard"],
    "椅子": ["chair", "office chair", "seat", "stool"],
    "桌子": ["table", "desk", "workstation"],
    "鞋": ["shoe", "shoes", "sneaker"],
    "箱子": ["box", "carton", "storage box"],
    "瓶子": ["bottle", "water bottle", "cup"],
    "插座": ["socket", "power outlet", "electrical outlet"],
    "水杯": ["cup", "mug", "water bottle"],
}


ROOM_CONTEXT_OBJECTS: dict[str, list[str]] = {
    "office": ["desk", "chair", "monitor", "keyboard", "printer", "cabinet", "trash can"],
    "entrance": ["door", "shoe", "shoe rack", "cabinet", "floor mat", "umbrella"],
    "living_room": ["sofa", "table", "remote control", "tv", "cabinet"],
    "kitchen": ["sink", "countertop", "fridge", "microwave", "cabinet", "bottle"],
}


COLOR_TERMS: dict[str, str] = {
    "红色": "red",
    "白色": "white",
    "黑色": "black",
    "蓝色": "blue",
    "绿色": "green",
    "黄色": "yellow",
    "灰色": "gray",
    "棕色": "brown",
    "橙色": "orange",
    "紫色": "purple",
}
