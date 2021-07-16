import os
import subprocess
import json

def create_lib(project_path):
    ret = subprocess.call(["powershell.exe", "IarBuild.exe {0} -build Debug".format(project_path)])
    return ret
    

def is_path_exist(path):
    print(path)
    if os.path.isdir(path):  
        True
    else:
        return False        


def read_config(json_path):
    config_file_path = json_path
    json_dict = dict()
    if os.path.exists(config_file_path):
        fs = open(config_file_path,'r+')
        data = fs.read()
        if data != '':
            json_dict = json.loads(data)
    else:
        if not os.path.exists('./config'):
            os.makedirs('./config')
        fs = open(config_file_path,'w')            
    fs.close()
    return json_dict


if __name__ == '__main__':
    config_dict = read_config('./config.json')   
    ret = create_lib(config_dict["gnss_lib_project_path"])    
    
    if ret != 0:
        print('build gnss lib error')
        exit(-1)
    ret = create_lib(config_dict["ins_lib_project_path"])
    if ret != 0:
        print('build ins lib error')
        exit(-2)
    dst_repo_local = os.path.join('../',config_dict["dst_repo_path"].split('/')[-1])
    if is_path_exist(dst_repo_local) == False:
        while True:
            fail_count = 0
            ret = subprocess.call(["powershell.exe", "cd ../ | git clone {0} -b {1}".format(config_dict["dst_repo_path"],config_dict["dst_repo_branch"])])
            if ret == 0 :
                break
            else:
                fail_count+= 1
                if fail_count > 5:
                    print('can\'t clone repo')
                    exit(-3)
    else:
        ret = subprocess.call(["powershell.exe", "cd {0} | git fetch".format(dst_repo_local)])
        if ret == True:
            ret = subprocess.call(["powershell.exe", "cd {0} | git pull origin {1}".format(dst_repo_local,config_dict["dst_repo_branch"])])
        if ret != 0:
            exit(-4)
    dst_repo_gnss_lib_path = os.path.join(dst_repo_local,config_dict["dst_gnss_lib_path"])
    dst_repo_ins_lib_path = os.path.join(dst_repo_local,config_dict["dst_ins_lib_path"])
    gnss_lib_folder_path, _ = os.path.split(dst_repo_gnss_lib_path)
    ins_lib_folder_path, _ = os.path.split(dst_repo_ins_lib_path)
    gnss_floder_path_to_add = ''
    ins_floader_path_to_add = ''
    if is_path_exist(gnss_lib_folder_path) == False:
        os.makedirs(gnss_lib_folder_path)
        gnss_floder_path_to_add = config_dict["dst_gnss_lib_path"][0:-1]
        print(gnss_floder_path_to_add)
    if is_path_exist(ins_lib_folder_path) == False:
        os.makedirs(ins_lib_folder_path)
        ins_floader_path_to_add = config_dict["dst_ins_lib_path"][0:-1]
        print(ins_floader_path_to_add)
        
    ret = subprocess.call(["powershell.exe", "cp {0} {1}".format(config_dict["gnss_lib_path"],dst_repo_gnss_lib_path)])
    if ret == 0:
        ret = subprocess.call(["powershell.exe", "cp {0} {1}".format(config_dict["ins_lib_path"],dst_repo_ins_lib_path)])   
    if ret != 0:
        print('copy lib error')

    commit_to_repo = input('ins401 commit:  ')
    commit_to_repo = "\"" + commit_to_repo + "\""
    
    subprocess.call(["powershell.exe", "cd {0} | git add {1} | git add {2} | git add {3} | git add {4} | git commit -m {5} "\
    .format(dst_repo_local,gnss_floder_path_to_add,ins_floader_path_to_add,config_dict["dst_gnss_lib_path"],config_dict["dst_ins_lib_path"],commit_to_repo)])
    subprocess.call(["powershell.exe", "cd {0} | git push origin master".format(dst_repo_local)])    