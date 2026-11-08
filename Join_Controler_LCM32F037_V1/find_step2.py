import json

path = r'C:\Users\DungTranBK\.gemini\antigravity\brain\8034d1f0-328f-4f3c-833b-c36f15aca20a\.system_generated\logs\transcript.jsonl'
with open(path, 'r', encoding='utf-8', errors='ignore') as f:
    for line in f:
        try:
            d = json.loads(line)
            idx = d.get('step_index')
            if 785 <= idx <= 795:
                print(f"Step {idx} ({d.get('type')}, {d.get('source')}):")
                print(str(d.get('content', ''))[:300])
                if 'tool_calls' in d:
                    print("Tool calls:", d['tool_calls'])
        except Exception:
            pass
