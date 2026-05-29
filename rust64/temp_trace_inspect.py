from pathlib import Path
lines = Path('trace62.txt').read_text(errors='ignore').splitlines()
for i, l in enumerate(lines, 1):
    if any(x in l for x in ['FIX-RESTORE-E8D7', 'IECIN-HANG', 'FASTLOAD-ENTRY', 'BRK-PATCH']):
        print('\\n---LINE', i, '---')
        for j in range(max(1, i-4), min(len(lines), i+6)+1):
            print(f'{j}: {lines[j-1]}')
