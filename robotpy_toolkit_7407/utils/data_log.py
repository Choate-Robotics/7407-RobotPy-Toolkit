import datetime
import inspect
import os


class Logger:
    """
    Custom logger utility for logging to console and a custom log file.
    """

    def __init__(
            self,
            debug: bool = False,
            filename: str = f"custom_logs/custom_logging_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log",
            to_file: bool = True
    ):

        self.filename: str = filename
        self.logfile = None
        self.debug_on = debug
        self.file_created = False
        self.to_file = to_file

        self.start_time = datetime.datetime.now()
        if not os.path.exists("./custom_logs"):
            os.mkdir(os.path.join(os.getcwd(), "custom_logs"))

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

        message = f"[{str(datetime.datetime.now() - self.start_time) + ']'} [{file_name + ':' + line_no + ']' : <19} \
        [{system + ']'  : <15} ~ {message  : <20}\n"

        if self.to_file:
            try:
                if self.file_created:
                    self.logfile = open(self.filename, "a")
                else:
                    self.logfile = open(self.filename, "x")
                    self.file_created = True
                self.logfile.write(
                    message
                )
                self.logfile.close()
            except FileNotFoundError:
                print("File Not Found")

        # lg.info(message, system, frame)
        # print(self.filename)
        print(message)

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
