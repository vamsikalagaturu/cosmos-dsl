import os

class Utils:
    def __init__(self):
        cwd = os.getcwd()
        self.ws = f'{cwd}/arm_actions/src/'
    
    def get_unique_file_name(self, file_name, dest_path):
        file_base, file_ext = os.path.splitext(file_name)
        dest_file_path = os.path.join(dest_path, file_name)

        # Check if the file already exists at the destination path
        if os.path.exists(dest_file_path):
            # Increment the number until a unique file name is found
            number = 1
            while True:
                new_file_name = f"{file_base}_{number}{file_ext}"
                new_file_path = os.path.join(dest_path, new_file_name)
                if not os.path.exists(new_file_path):
                    return new_file_name
                number += 1
        else:
            # File doesn't exist, return the original file name
            return file_name

    def write_to_file(self, result: str, template_name: str):
        fname = template_name.split('/')[-1]
        fname = fname.strip('.jinja2')
        fname = self.ws + self.get_unique_file_name(fname, self.ws)

        with open(fname, 'w+') as f:
            f.write(result)

if __name__ == "__main__":
    utils = Utils()