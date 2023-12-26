import json
import os
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional

from openai import OpenAI
from tavily import TavilyClient
from utils import Logger

from mnlm.client.gpt_control.robot_arm import (OperationSequenceGenerator,
                                               RobotArmControlClient,
                                               SimulatedRobotArmControl)


class Tool(ABC):
    def __init__(self, name: str, logger: Logger, verbose: bool = False):
        self.name = name
        self.logger = logger
        self.verbose = verbose

    @abstractmethod
    def get_signature(self) -> Dict[str, Any]:
        pass

    def __call__(self, *args, **kwargs):
        if self.verbose:
            self.logger.info(
                f"Executing tool {self.name} with args {args} and kwargs {kwargs}"
            )
        result = self.execute(*args, **kwargs)
        if self.verbose:
            self.logger.info(f"Result from tool {self.name}: {result}")
        return result

    @abstractmethod
    def execute(self, *args, **kwargs) -> str:
        pass


class SearchEngine(Tool):
    def __init__(self, name: str, logger: Logger, verbose: bool = False):
        super().__init__(name=name, logger=logger, verbose=verbose)
        self.client = TavilyClient(api_key=os.getenv("TAVILY_API_KEY"))

    def get_signature(self) -> Dict[str, Any]:
        signature = {
            "type": "function",
            "function": {
                "name": "search_engine",
                "description": "Get information from the search engine",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "The query to search for",
                        },
                    },
                    "required": ["query"],
                },
            },
        }
        return signature

    def execute(self, query: str) -> str:
        response = self.client.search(query=query)
        contexts = [
            {"url": obj["url"], "content": obj["content"]}
            for obj in response["results"]
        ]
        return json.dumps(contexts)


class RobotArmController(Tool):
    def __init__(
        self,
        name: str,
        logger: Logger,
        gpt_client: Optional[OpenAI] = None,
        simulation: bool = True,
        verbose: bool = False,
    ):
        super().__init__(name=name, logger=logger, verbose=verbose)
        api_document_path = "~/source/ml-prototype/data/knowledge/robot_arm.md"
        self.operation_generator = OperationSequenceGenerator(
            api_document_path=api_document_path,
            gpt_client=gpt_client,
            logger=logger,
        )
        self.simulation = simulation
        self.knowledge = f"""

        """
        if simulation:
            self.robot_arm_control = SimulatedRobotArmControl()
        else:
            self.robot_arm_control = RobotArmControlClient()

    def get_signature(self) -> Dict[str, Any]:
        signature = {
            "type": "function",
            "function": {
                "name": "robot_arm",
                "description": "Operate on a robotic arm, you would send the instruction in text.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "instruction": {
                            "type": "string",
                            "description": """
                                Oral command to the robot arm. It can generate a sequence of operations in one batch.
                            """,
                        },
                    },
                    "required": ["instruction"],
                },
            },
        }
        return signature

    def execute(self, instruction: str) -> str:
        try:
            # Execute operations using the chosen mode (simulation or real)
            operations_json = self.operation_generator.translate_prompt_to_sequence(
                prompt=instruction
            )
            if self.verbose:
                self.logger.info(f"Robot arm command: {operations_json}.")
            self.robot_arm_control.execute_operations(operations_json)
            return "Executed robot arm operations successfully."
        except Exception as e:
            self.logger.error(f"Error executing robot arm operations: {e}")
            return f"Error: {e}"


def init_tools(logger: Logger, verbose: bool = False):
    tools = {
        # "search_engine": SearchEngine(
        #     name="search_engine", logger=logger, verbose=verbose
        # ),
        "robot_arm": RobotArmController(
            name="robot_arm",
            logger=logger,
            verbose=verbose,
            simulation=False,
        ),
    }
    return tools


def use_tool(
    tools: Dict[str, Any], action: Dict[str, Any], logger: Logger, verbose: bool
) -> str:
    """Parse the function call action and execute the function.

    Args:
        action (str): The json object of assistant's function call action.
    """
    func_name = action["function"]["name"]
    arguments = json.loads(action["function"]["arguments"])
    if func_name not in tools:
        return "Function not found"
    output = tools[func_name](**arguments)
    if verbose:
        logger.info(f"Output from function {func_name}: {output}")
    return output
