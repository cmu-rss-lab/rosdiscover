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
localRepoPath = sys.argv[4]
out_dir = sys.argv[5]

g = Git(localRepoPath)
r = Repo(localRepoPath)
distro = "indigo"
repoUrl = r.remotes[0].url

c = r.commit(pre_bug_commit)
c_date = c.authored_datetime.isoformat()
g.checkout(pre_bug_commit)
neededPackages = gp.get_packages(localRepoPath)
pc = ' '.join(neededPackages)
command = rgtm_dir+"/rosinstall_generator_tm.sh '"+c_date+"' "+distro+" "+pc+" --deps > "+out_dir+"/pre_bug.rosinstall"
print(command)
os.system(command)

file1 = open(out_dir+"/pre_bug.rosinstall", "a")  # append mode
file1.write("- git:\n    local-name: repo\n    uri:  "+repoUrl+"\n    version: "+pre_bug_commit)
file1.close()

 
c = r.commit(bug_commit)
c_date = c.authored_datetime.isoformat()
g.checkout(bug_commit)
neededPackages = gp.get_packages(localRepoPath)
pc = ' '.join(neededPackages)
command = rgtm_dir+"/rosinstall_generator_tm.sh '"+c_date+"' "+distro+" "+pc+" --deps > "+out_dir+"/bug.rosinstall"
print(command)
os.system(command)
file2 = open(out_dir+"/bug.rosinstall", "a")  # append mode
file2.write("- git:\n    local-name: repo\n    uri:  "+repoUrl+"\n    version: "+bug_commit)
file2.close()


c = r.commit(bug_fix_commit)
c_date = c.authored_datetime.isoformat()
g.checkout(bug_fix_commit)
neededPackages = gp.get_packages(localRepoPath)
pc = ' '.join(neededPackages)
command = rgtm_dir+"/rosinstall_generator_tm.sh '"+c_date+"' "+distro+" "+pc+" --deps > "+out_dir+"/bug_fix.rosinstall"
print(command)
os.system(command)
file3 = open(out_dir+"/bug_fix.rosinstall", "a")  # append mode
file3.write("- git:\n    local-name: repo\n    uri:  "+repoUrl+"\n    version: "+bug_fix_commit)
file3.close()