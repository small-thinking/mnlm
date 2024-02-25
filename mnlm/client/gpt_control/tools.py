import json
import os
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional

from command_indexer import InstructionIndexer  # type: ignore
from openai import OpenAI
from robot_arm import (
    OperationSequenceGenerator,
    RobotArmControl,
    RobotArmControlClient,
    SimulatedRobotArmControl,
)
from tavily import TavilyClient  # type: ignore
from utils import Logger  # type: ignore


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
        use_rag: bool = False,
        verbose: bool = False,
    ):
        super().__init__(name=name, logger=logger, verbose=verbose)
        # The api doc is under client/knowledge/robot_arm.md
        api_document_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), "knowledge", "robot_arm.md"
        )
        self.use_rag = use_rag
        if self.use_rag:
            self.indexer = InstructionIndexer()
            self.indexer.load_index_and_data()
        else:
            self.operation_generator = OperationSequenceGenerator(
                api_document_path=api_document_path,
                gpt_client=gpt_client,
                logger=logger,
            )

        self.simulation = simulation
        self.knowledge = f"""

        """
        self.robot_arm_control: RobotArmControl = (
            SimulatedRobotArmControl() if simulation else RobotArmControlClient()
        )

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
            if self.use_rag:
                operations_json = self.indexer.retrieve_operation_sequences(
                    instruction=instruction
                )
            else:
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


def init_tools(
    logger: Logger,
    verbose: bool = False,
    use_dummy_robot_arm_server: bool = False,
    use_rag: bool = False,
) -> Dict[str, Any]:
    """Initialize the tools for the assistant.

    Args:
        logger (Logger): The logger.
        verbose (bool): If True, prints verbose output.
        use_dummy_robot_arm_server (bool): If True, use the simulation mode.
    """
    tools = {
        # "search_engine": SearchEngine(
        #     name="search_engine", logger=logger, verbose=verbose
        # ),
        "robot_arm": RobotArmController(
            name="robot_arm",
            logger=logger,
            verbose=verbose,
            use_rag=use_rag,
            simulation=use_dummy_robot_arm_server,
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
