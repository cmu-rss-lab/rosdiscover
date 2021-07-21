import sys
from pathlib import Path
import get_packages as gp
import xml.etree.ElementTree as ET
import os
import yaml
from yaml.loader import SafeLoader

from pathlib import Path


from git import *
rgtm_dir = "~/rosinstall_generator_time_machine"
tempdir = "./ros-temp/"

with open(sys.argv[1]) as f:
    data = yaml.load(f, Loader=SafeLoader)

docker_dir = sys.argv[2]

repo_url = data["repo-url"]
repo_name = data["repo-name"]
pre_bug_commit = data["pre-bug-commit"]
bug_commit = data["bug-commit"]
bug_fix_commit = data["bug-fix-commit"]
bug_id = data["bugid"]
localReposPath = tempdir	+ "repos/"
out_dir = tempdir	+ bug_id + "/time/"
distro = data["ros-distro"]

if not os.path.exists(out_dir):
	Path(out_dir).mkdir(parents=True)
if not os.path.exists(localReposPath):
	Path(localReposPath).mkdir(parents=True)

localRepoPath = localReposPath + repo_name
if not os.path.exists(localRepoPath):
	print("Cloning repo into " + localRepoPath)
	p = Git(localReposPath).clone(repo_url)
	print("Done cloning repo")
g = Git(localRepoPath)
r = Repo(localRepoPath)
repoUrl = r.remotes[0].url

if not os.path.exists(out_dir+"/pre_bug.rosinstall"):
	c = r.commit(pre_bug_commit)
	c_date = c.authored_datetime.isoformat()
	g.checkout(pre_bug_commit)
	neededPackages = gp.get_packages(localRepoPath)
	pc = ' '.join(neededPackages)
	command = "yes | "+rgtm_dir+"/rosinstall_generator_tm.sh '"+c_date+"' "+distro+" "+pc+" --deps > "+out_dir+"/pre_bug.rosinstall"
	os.system(command)
	file1 = open(out_dir+"/pre_bug.rosinstall", "a")  # append mode
	file1.write("- git:\n    local-name: repo\n    uri:  "+repoUrl+"\n    version: "+pre_bug_commit)
	file1.close()

if not os.path.exists(out_dir+"/bug.rosinstall"):
	c = r.commit(bug_commit)
	c_date = c.authored_datetime.isoformat()
	g.checkout(bug_commit)
	neededPackages = gp.get_packages(localRepoPath)
	pc = ' '.join(neededPackages)
	command = "yes | "+rgtm_dir+"/rosinstall_generator_tm.sh '"+c_date+"' "+distro+" "+pc+" --deps > "+out_dir+"/bug.rosinstall"
	os.system(command)
	file2 = open(out_dir+"/bug.rosinstall", "a")  # append mode
	file2.write("- git:\n    local-name: repo\n    uri:  "+repoUrl+"\n    version: "+bug_commit)
	file2.close()

if not os.path.exists(out_dir+"/bug_fix.rosinstall"):
	c = r.commit(bug_fix_commit)
	c_date = c.authored_datetime.isoformat()
	g.checkout(bug_fix_commit)
	neededPackages = gp.get_packages(localRepoPath)
	pc = ' '.join(neededPackages)
	command = "yes | "+rgtm_dir+"/rosinstall_generator_tm.sh '"+c_date+"' "+distro+" "+pc+" --deps > "+out_dir+"/bug_fix.rosinstall"
	os.system(command)
	file3 = open(out_dir+"/bug_fix.rosinstall", "a")  # append mode
	file3.write("- git:\n    local-name: repo\n    uri:  "+repoUrl+"\n    version: "+bug_fix_commit)
	file3.close()

print(docker_dir)
if docker_dir != 'none':
	dockerCmd = "docker build . -t "+ bug_id + " --build-arg DISTRO="+distro+" --build-arg DIRECTORY="+out_dir+" -f "+docker_dir
	print(dockerCmd)
	res = os.system(dockerCmd)