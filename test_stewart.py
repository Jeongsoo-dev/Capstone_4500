import requests
import time

URL = "http://192.168.4.1/move"

test_cases = [
    {"pitch": 0.0, "roll": 0.0},
    {"pitch": 10.0, "roll": 0.0},
    {"pitch": -10.0, "roll": 0.0},
    {"pitch": 0.0, "roll": 10.0},
    {"pitch": 0.0, "roll": -10.0},
    {"pitch": 10.0, "roll": 10.0},
    {"pitch": -10.0, "roll": -10.0},
    {"pitch": 15.0, "roll": -15.0},
    {"pitch": -15.0, "roll": 15.0}
]

for case in test_cases:
    print(f"Sending: pitch={case['pitch']}, roll={case['roll']}")
    try:
        response = requests.post(URL, json=case, timeout=2)
        print("Response:", response.text)
    except Exception as e:
        print("Failed:", e)
    time.sleep(3)
