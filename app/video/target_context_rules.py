"""Small, explicit priors used only for candidate-region suggestions."""

TARGET_CONTEXT_RULES = {
    "手机": {
        "likely_objects": [
            "table", "desk", "coffee table", "sofa", "bed", "charger", "cable",
            "桌", "茶几", "沙发", "床", "充电器", "线",
        ],
        "likely_regions": ["桌面", "茶几", "沙发", "床边", "充电区域"],
        "search_hint": "优先搜索桌面、沙发、床边和充电线附近。",
    },
    "水杯": {
        "likely_objects": [
            "table", "desk", "coffee table", "kitchen", "sink", "桌", "茶几", "厨房",
        ],
        "likely_regions": ["桌面", "茶几", "厨房台面", "水槽附近"],
        "search_hint": "优先搜索桌面、茶几和厨房台面。",
    },
    "充电器": {
        "likely_objects": [
            "outlet", "desk", "bedside table", "cable", "power strip",
            "插座", "书桌", "床头柜", "电线", "插排",
        ],
        "likely_regions": ["插座附近", "书桌", "床头柜", "电源线附近"],
        "search_hint": "优先搜索插座、电源线、书桌和床头柜附近。",
    },
    "钥匙": {
        "likely_objects": [
            "table", "door", "cabinet", "shoe cabinet", "桌", "门", "柜", "鞋柜",
        ],
        "likely_regions": ["玄关", "桌面", "鞋柜", "门口"],
        "search_hint": "优先搜索玄关、门口、桌面和鞋柜附近。",
    },
    "鞋子": {
        "likely_objects": [
            "door", "floor", "shoe cabinet", "entrance", "门", "地板", "鞋柜", "玄关",
        ],
        "likely_regions": ["玄关", "门口", "地面", "鞋柜附近"],
        "search_hint": "优先搜索玄关门口、地面和鞋柜附近。",
    },
    "厕所": {
        "likely_objects": [
            "restroom sign",
            "toilet sign",
            "bathroom sign",
            "wc sign",
            "male sign",
            "female sign",
            "accessible sign",
            "sign",
            "door",
            "corridor",
            "指示牌",
            "标志",
            "男女标识",
            "无障碍标识",
            "门",
            "走廊",
        ],
        "likely_regions": ["带卫生间标识的门", "走廊门口", "指示牌指向区域"],
        "search_hint": (
            "视频中没有直接确认厕所；优先回到同时出现门和指示牌的时间片，"
            "检查门牌、男女/无障碍标识或指示牌方向。"
        ),
    },
}
