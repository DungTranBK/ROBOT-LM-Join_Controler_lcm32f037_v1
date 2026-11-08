import json

path = r'C:\Users\DungTranBK\.gemini\antigravity\brain\8034d1f0-328f-4f3c-833b-c36f15aca20a\.system_generated\logs\transcript.jsonl'
with open(path, 'r', encoding='utf-8', errors='ignore') as f:
    for line in f:
        try:
            d = json.loads(line)
            idx = d.get('step_index')
            if idx in [378, 379, 380, 381, 382]:
                print(f"Step {idx} ({d.get('type')}):")
                print(str(d.get('content', ''))[:400])
                if 'tool_calls' in d:
                    print("Tool calls:", d['tool_calls'])
        except Exception:
            pass
