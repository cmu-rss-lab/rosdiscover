import sys
from pathlib import Path
import get_packages as gp
import xml.etree.ElementTree as ET
import os

from git import *
rgtm_dir = "~/rosinstall_generator_time_machine"

pre_bug_commit = sys.argv[1]
bug_commit = sys.argv[2]
bug_fix_commit = sys.argv[3]
repo = sys.argv[4]

g = Git(repo)
r = Repo(repo)

c = r.commit(pre_bug_commit)
c_date = c.authored_datetime.isoformat()
distro = "indigo"
g.checkout(pre_bug_commit)
neededPackages = gp.get_packages(repo)
pc = ' '.join(neededPackages)
os.system(f"{rgtm_dir}/rosinstall_generator_tm.sh '{c_date}' {distro} {pc} --deps > pre_bug.rosinstall")

 
