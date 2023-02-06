import datetime
import inspect
import os

import robotpy_toolkit_7407.utils.logger as lg


class Logger:
    """
    Custom logger utility for logging to console and a custom log file.
    """
    def __init__(
        self,
        debug: bool = False,
        use_file: bool = True,
        filename: str = f"custom_logs/custom_logging_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log",
    ):
        self.filename: str = filename
        self.logfile = None
        self.debug_on = debug
        self.file_on = use_file

        self.start_time = datetime.datetime.now()

    def log(self, system: str, message: str):
        """
        Log a message to the console and a custom log file.
        Args:
            system: The system that the message is coming from.
            message: The message to log.

        Returns:
            None
        """
        frame = inspect.stack()[1].frame
        file_name = os.path.basename(frame.f_code.co_filename)
        line_no = str(frame.f_lineno)

        if self.file_on:
            self.logfile = open(self.filename, "a")
            self.logfile.write(
                f"[{str(datetime.datetime.now() - self.start_time) + ']'} [{file_name + ':' + line_no + ']' : <19} [{system + ']'  : <15} ~ {message  : <20}\n"
            )
            self.logfile.close()

        lg.info(message, system, frame)

    def debug(self, system: str, message: str):
        """
        Log a debug message to the console and a custom log file.
        Args:
            system: The system that the message is coming from.
            message: The message to log.

        Returns:
            None
        """
        if self.debug_on:
            self.log(system, message)

    def close(self):
        if self.logfile:
            self.logfile.close()
