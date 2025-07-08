import subprocess
import os
import logging
import logfunc

def clone_repo(repo_url_path,local_path,commit_message):
    try:
        if not os.path.isdir(local_path):
            logging.info(f"Folder {local_path} does not exist. Cloning repo..")
            subprocess.run(["git","clone", repo_url_path,local_path], check=True)
            with open("/ProtoDevelopementpi/update_ota/ota_update_flag.txt","w") as f:
                f.write("cloned\n")
            logging.info("Repo cloned successfully.....   ")
        else:
        
            git_folder = os.path.join(local_path,".git")
            if os.path.isdir(git_folder):
               logging.info(f"Folder {local_path} exists and is a git repo.pulling latest changes...")
               subprocess.run(["git","-C",local_path,"pull"], check = True)
               with open("/ProtoDevelopementpi/update_ota/ota_update_flag.txt","w") as f:
                   f.write("pulled\n")
               logging.info("Repo updated Successfully")
            
            else:
                logging.info(f"Folder {local_path} exists but is NOT a git repo.Cannot pull updates.")
                return
        result= subprocess.run(["git","-C", local_path, "status", "--porcelain"],stdout=subprocess.PIPE, text=True)
      
        if result.stdout.strip() =="":
            logging.info("Nothing to commit - working tree clean")
        else:
            subprocess.run(["git","-C",local_path, "add", "."],check=True)
            subprocess.run(["git","-C",local_path, "commit" ,"-m" ,commit_message],check=True)
            subprocess.run(["git","-C",local_path, "push"],check=True)
            logging.info("Change pushed successfully")
    except subprocess.CalledProcessError as e:
            logging.info(f"Git Operation failed: {e}")

    firmware_script = os.path.join(locate_path,"Firmware_ota.py")
    if os.path.isfile(firmware_script):

        logging.info(f"Running firmware update: {firmware_script}")
        try:
            subprocess.run(["python3", firmware_script],check=True)
        except subprocess.CalledProcessError as e:
            logging.info(f"Error running firmware script:{e}")
    else:
        logging.info(f"Firmware script not found: {firmware_script}")
        
#usage
repo_url="https://github.com/Soumyasri2000/upadate_ota.git"
local_path="/ProtoDevelopementpi/update_ota"
locate_path="/ProtoDevelopementpi/"
commit_message = "OTA update pushed from Pi"

clone_repo(repo_url,local_path, commit_message)

