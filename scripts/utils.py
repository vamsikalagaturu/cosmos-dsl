import os
from jinja2 import Template

class Utils:
    def __init__(self):
        self.ws = 'ws/'

    def get_new_folder(self, dest: str) -> str:
        # check how many folders are in the path
        folder_names = [name for name in os.listdir(dest) if os.path.isdir(os.path.join(dest, name))]
        # check if empty
        if not folder_names:
            new_folder = dest + 'test_1/'
        else:
            folder_names.sort()
            last_folder = folder_names[-1]
            new_folder = dest + 'test_' + str(int(last_folder.split('_')[-1]) + 1) + '/'

        return new_folder
    
    def get_fname_from_template(self, template_name: str) -> str:
        # get the path from the template file name
        path = template_name.split('/')[0:-1]

        return '/'.join(path)

    def write_to_file(self, result: str, template_name: str):
        new_folder = self.get_new_folder(self.ws)
        fname = self.get_fname_from_template(template_name)

        # create the new folder
        os.makedirs(new_folder + fname, exist_ok=True)

        with open(new_folder + template_name.strip('.jinja2'), 'w+') as f:
            f.write(result)