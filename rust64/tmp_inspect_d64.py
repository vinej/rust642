from pathlib import Path
SECTOR_SIZE=256
SECTORS_BEFORE=[0,0,21,42,63,84,105,126,147,168,189,210,231,252,273,294,315,336,357,376,395,414,433,452,471,490,508,526,544,562,580,598,615,632,649,666,683,700,717,734,751]

def get_sector(track, sector):
    off=(SECTORS_BEFORE[track] + sector) * SECTOR_SIZE
    return data[off:off + SECTOR_SIZE]

data=Path('arkanoid.d64').read_bytes()
track=18
sector_num=1
for _ in range(20):
    s=get_sector(track, sector_num)
    for i in range(8):
        off=i*32
        ft=s[off+2]
        if ft & 0x0F:
            name=''.join(chr(b & 127) if b >= 32 else '.' for b in s[off+5:off+21])
            size=s[off+31]*256+s[off+30]
            print('ENTRY', name, 'type', hex(ft), 'start_track', s[off+3], 'start_sector', s[off+4], 'size_sectors', size)
            tr=s[off+3]; se=s[off+4]
            file_bytes=[]
            t=tr; sec=se
            for _ in range(1000):
                st=get_sector(t, sec)
                nt=st[0]; ns=st[1]
                if nt==0:
                    last=ns
                    file_bytes.extend(st[2:last+1])
                    break
                file_bytes.extend(st[2:256])
                t=nt; sec=ns
            print('file_len', len(file_bytes), 'load_addr', file_bytes[:2], 'first_bytes', file_bytes[2:12])
    next_t=s[0]
    next_s=s[1]
    if next_t==0:
        break
    track=next_t
    sector_num=next_s
