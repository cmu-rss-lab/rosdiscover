import sys
from pathlib import Path
import xml.etree.ElementTree as ET

repo = sys.argv[1]
print(repo)

definedPackages = set()
usedPackages = set()

for path in Path(repo).rglob('package.xml'):
	mytree = ET.parse(path)
	myroot = mytree.getroot()
	for x in myroot.findall('name'):
		definedPackages.add(x.text)
	for x in myroot.findall('build_depend'):
		usedPackages.add(x.text)
	
neededPackages = usedPackages - definedPackages
print(' '.join(neededPackages))
