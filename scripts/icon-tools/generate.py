from pathlib import Path
from PIL import Image
import subprocess, shutil, xml.etree.ElementTree as ET
root=Path(__file__).resolve().parents[2]
a=root/'oxifoc-ui/assets'
subprocess.run([shutil.which('rsvg-convert') or 'rsvg-convert',str(a/'oxifoc-icon.svg'),'-o',str(a/'oxifoc-icon.png')],check=True)
im=Image.open(a/'oxifoc-icon.png').convert('RGBA')
(a/'oxifoc-icon-64.rgba').write_bytes(im.resize((64,64),Image.Resampling.LANCZOS).tobytes())
r=root/'experiments/ble-mobile/android-res'
for density,size in [('mdpi',48),('hdpi',72),('xhdpi',96),('xxhdpi',144),('xxxhdpi',192)]:
    d=r/f'mipmap-{density}';d.mkdir(parents=True,exist_ok=True);im.resize((size,size),Image.Resampling.LANCZOS).save(d/'ic_launcher.png')
d=r/'drawable';d.mkdir(parents=True,exist_ok=True)
fg=Image.new('RGBA',(432,432));fg.alpha_composite(im.resize((288,288),Image.Resampling.LANCZOS),(72,72));fg.save(d/'oxifoc_foreground.png')
paths=[e.attrib['d'] for e in ET.parse(a/'oxifoc-icon.svg').getroot() if e.tag.endswith('path')]
mono='<vector xmlns:android="http://schemas.android.com/apk/res/android" android:width="108dp" android:height="108dp" android:viewportWidth="108" android:viewportHeight="108"><group android:scaleX="0.140625" android:scaleY="0.140625" android:translateX="18" android:translateY="18">'+''.join(f'<path android:fillColor="#FFFFFFFF" android:pathData="{p}"/>' for p in paths)+'<path android:fillColor="#FFFFFFFF" android:fillType="evenOdd" android:pathData="M307,256 A51,51 0,1 1,205,256 A51,51 0,1 1,307,256 M291,256 A35,35 0,1 0,221,256 A35,35 0,1 0,291,256"/></group></vector>'
(d/'oxifoc_monochrome.xml').write_text(mono)
for version in [26,33]:
    d=r/f'mipmap-anydpi-v{version}';d.mkdir(exist_ok=True)
    xml='<adaptive-icon xmlns:android="http://schemas.android.com/apk/res/android"><background android:drawable="@color/oxifoc_background"/><foreground android:drawable="@drawable/oxifoc_foreground"/>'
    if version==33:xml+='<monochrome android:drawable="@drawable/oxifoc_monochrome"/>'
    (d/'ic_launcher.xml').write_text(xml+'</adaptive-icon>\n')
d=r/'values';d.mkdir(exist_ok=True);(d/'oxifoc.xml').write_text('<resources><color name="oxifoc_background">#0B1321</color></resources>\n')
