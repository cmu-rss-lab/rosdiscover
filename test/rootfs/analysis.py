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
out_dir = sys.argv[5]

g = Git(repo)
r = Repo(repo)
distro = "indigo"

c = r.commit(pre_bug_commit)
c_date = c.authored_datetime.isoformat()
g.checkout(pre_bug_commit)
neededPackages = gp.get_packages(repo)
pc = ' '.join(neededPackages)
command = rgtm_dir+"/rosinstall_generator_tm.sh '"+c_date+"' "+distro+" "+pc+" --deps > "+out_dir+"/pre_bug.rosinstall"
print(command)
os.system(command)

 
c = r.commit(bug_commit)
c_date = c.authored_datetime.isoformat()
g.checkout(bug_commit)
neededPackages = gp.get_packages(repo)
pc = ' '.join(neededPackages)
command = rgtm_dir+"/rosinstall_generator_tm.sh '"+c_date+"' "+distro+" "+pc+" --deps > "+out_dir+"/bug.rosinstall"
print(command)
os.system(command)


c = r.commit(bug_fix_commit)
c_date = c.authored_datetime.isoformat()
g.checkout(bug_fix_commit)
neededPackages = gp.get_packages(repo)
pc = ' '.join(neededPackages)
command = rgtm_dir+"/rosinstall_generator_tm.sh '"+c_date+"' "+distro+" "+pc+" --deps > "+out_dir+"/bug_fix.rosinstall"
print(command)
os.system(command)