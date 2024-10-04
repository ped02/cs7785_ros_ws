from abc import abstractmethod

from typing import Any


class Controller:
    @abstractmethod
    def execute(self, error, current_time_sec, previous_time_sec) -> Any:
        raise NotImplementedError('execute needs to be implemented')
