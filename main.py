import json

path = r"/media/serlini/data3/dream/annotators/IntentCatcherTransformers/msg/data.json"

with open(path) as f:
    print(json.load(f))