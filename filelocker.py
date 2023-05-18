import os


class FileLock:

    def __init__(self, lock_file_path) -> None:
        self.lock_file_path = lock_file_path

        self.locked_suffix = '.locked'
        self.unlocked_suffix = ''

        self.is_locked = False
        if(not os.path.isfile(self.lock_file_path + self.locked_suffix)):
            fp = open(self.lock_file_path, 'wb')
            fp.close()
            # os.remove(self.lock_file_path + self.locked_suffix)

        pass

    def acquire(self):
        if(not os.path.isfile(self.lock_file_path + self.locked_suffix)):
            os.rename(self.lock_file_path,
                      self.lock_file_path + self.locked_suffix)

        self.is_locked = True

        pass

    def release(self):
        if(os.path.isfile(self.lock_file_path + self.locked_suffix)):
            os.rename(self.lock_file_path + self.locked_suffix,
                      self.lock_file_path + self.unlocked_suffix)

        self.is_locked = False
        pass

    def is_lock(self):

        return os.path.isfile(self.lock_file_path + self.locked_suffix)
