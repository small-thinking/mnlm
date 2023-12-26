import os
import time
from typing import Any, Dict, List, Optional

from dotenv import load_dotenv
from openai import OpenAI
from openai.types.beta import Assistant, Thread
from openai.types.beta.threads import Run, ThreadMessage
from tools import *
from utils import Logger
from voice import *


def create_assistant(
    client: OpenAI, tools: Dict[str, Tool], logger: Logger, verbose: bool = False
) -> Assistant:
    """Create a GPT based orchestration assistant.

    Parameters:
        client (OpenAI): The OpenAI client.
        tools (Dict[str, Tool]): The tools dictionary.
        logger (Logger): The logger.
        verbose (bool): If True, prints verbose output.

    Returns:
        Assistant: The assistant.
    """
    if not client:
        client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))
    tool_signatures: List[Dict[str, Any]] = [
        tool.get_signature() for tool in tools.values()
    ]
    assistant = client.beta.assistants.create(
        name="Voice Robot Controller",
        instructions="You have a brain and a robot arm, and you receive voice command from the human being, and respond accordingly.",
        tools=tool_signatures,
        model="gpt-3.5-turbo-1106",
    )
    if verbose:
        logger.info(f"Assistant created: {assistant}")
    return assistant


def create_run(
    client: OpenAI,
    assistant_id: str,
    thread_id: str,
    instructions: str,
    logger: Logger,
    verbose: bool,
) -> Run:
    """Creates a run for the assistant.

    Parameters:
        client (OpenAI): The OpenAI client.
        assistant_id (str): The assistant ID.
        thread_id (str): The thread ID.
        instructions (str): The instructions.
        logger (Logger): The logger.
        verbose (bool): If True, prints verbose output.

    Returns:
        Run: The run.
    """
    run = client.beta.threads.runs.create(
        thread_id=thread_id,
        assistant_id=assistant_id,
        instructions=instructions,
    )
    if verbose:
        logger.info(f"Run created: {run}")
    return run


def wait_for_run(
    client: OpenAI,
    thread_id: str,
    run_id: str,
    tools: Dict[str, Any],
    logger: Logger,
    verbose: bool = False,
) -> str:
    """
    Waits for the run to complete.

    Parameters:
        client (OpenAI): The OpenAI client.
        thread_id (str): The thread ID.
        run_id (str): The run ID.
        tools (Dict[str, Any]): The tools dictionary.
        logger (Logger): The logger.
        verbose (bool): If True, prints verbose output.

    Returns:
        str: The response from the run.
    """
    while True:
        # Retrieve the run status.
        run = client.beta.threads.runs.retrieve(
            thread_id=thread_id,
            run_id=run_id,
        )
        if run.status == "completed":
            messages = client.beta.threads.messages.list(
                thread_id=thread_id, limit=2, order="desc"
            )
            response = ""
            for message in messages:
                response += message.content[0].text.value
                print(f"{message.role.capitalize()}: {message.content[0].text.value}")
                break
            return response
        elif run.status == "in_progress" or run.status == "queued":
            if verbose:
                logger.info(
                    f"Run in progress: {run.required_action}, {run.status}, {run.tools if hasattr(run, 'tools') else ''}\n\n"
                )
            time.sleep(2)
        elif run.status == "requires_action":
            if verbose:
                logger.info(f"Run requires action.")
            required_actions = run.required_action.submit_tool_outputs.model_dump()
            if verbose:
                logger.info(required_actions)
            tools_outputs = []
            for action in required_actions["tool_calls"]:
                output = use_tool(
                    tools=tools, action=action, logger=logger, verbose=verbose
                )
                if verbose:
                    logger.info(f"Output from {action['id']} tool: {output}")
                tools_outputs.append(
                    {
                        "tool_call_id": action["id"],
                        "output": output,
                    }
                )
            # Submit the tool outputs to Assistant API
            if verbose:
                logger.info(f"Submitting tool outputs: {tools_outputs}")
            client.beta.threads.runs.submit_tool_outputs(
                thread_id=thread_id,
                run_id=run_id,
                tool_outputs=tools_outputs,
            )
        else:
            logger.error(f"Run failed: {run.status}\n\n")
            break


def cleanup(
    client: OpenAI, logger: Optional[Logger] = None, verbose: bool = False
) -> None:
    pass


def start_conversation(
    verbose: bool,
    nudge_user: bool,
    use_voice_input: bool,
    use_voice_output: bool,
    logger: Logger,
) -> None:
    load_dotenv()
    client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))
    tools = init_tools(logger=logger, verbose=verbose)
    assistant = create_assistant(
        client=client, tools=tools, logger=logger, verbose=verbose
    )
    thread = client.beta.threads.create()

    if verbose:
        logger.info(f"Thread created: {thread}")

    while True:
        if use_voice_input:
            user_input = generate_transcription(verbose=False)
            print(f"User: {user_input}")
        else:
            print("User: ", end="")
            user_input = input()

        if user_input.lower() == "exit":
            break

        if not user_input and nudge_user:
            logger.warning("No input detected. Please speak clearly.")
            continue

        message = client.beta.threads.messages.create(
            thread_id=thread.id,
            role="user",
            content=user_input,
        )
        if verbose:
            logger.info(f"Message created: {message}")

        run = create_run(
            client=client,
            assistant_id=assistant.id,
            thread_id=thread.id,
            instructions=user_input,
            logger=logger,
            verbose=verbose,
        )
        response = wait_for_run(
            client=client,
            thread_id=thread.id,
            run_id=run.id,
            tools=tools,
            logger=logger,
            verbose=verbose,
        )

        if use_voice_output:
            speak(text=response, client=client)

    cleanup(client=client, verbose=verbose, logger=logger)


if __name__ == "__main__":
    verbose = True
    nudge_user = True
    use_voice_input = True  # Set to True to enable voice input
    use_voice_output = True  # Set to True to enable voice output
    logger = Logger(__name__)
    start_conversation(
        verbose=verbose,
        nudge_user=nudge_user,
        use_voice_input=use_voice_input,
        use_voice_output=use_voice_output,
        logger=logger,
    )
