"""Test of the robot arm.

Run test: poetry run pytest tests/gpt_control/test_robot_arm.py
"""

import json
import os
from unittest.mock import MagicMock, patch

import pytest
from openai.types.chat import ChatCompletion, ChatCompletionMessage
from openai.types.chat.chat_completion import Choice

from mnlm.client.gpt_control.robot_arm import (
    OperationSequenceGenerator,
    SimulatedRobotArmControl,
)


# Test for OperationSequenceGenerator
def test_generate_operation():
    # Fake gpt_client in OperationSequenceGenerator.
    with patch.dict(os.environ, {"OPENAI_API_KEY": "your_mocked_api_key"}):
        generator = OperationSequenceGenerator(api_document_path="dummy_path.md")
        operation_json = generator.generate_operation("set_rgb_light", R=255, G=0, B=0)
        assert json.loads(operation_json) == {
            "operation": "set_rgb_light",
            "parameters": {"R": 255, "G": 0, "B": 0},
        }


@pytest.mark.parametrize(
    "prompt, expected_response",
    [
        (
            "Turn on the RGB light with red color.",
            """[{"operation": "set_rgb_light", "parameters": {"R": 255, "G": 0, "B": 0}}]""",
        ),
        (
            "Move servo 1 to 90 degrees.",
            """[{"operation": "move_single_servo", "parameters": {"id": 1, "angle": 90, "time": 500}}]""",
        ),
        (
            "Move servo 1 to 90 degrees and then move servo 2 to 120 degrees.",
            """[
            {"operation": "move_single_servo", "parameters": {"id": 1, "angle": 90, "time": 500}},
            {"operation": "move_single_servo", "parameters": {"id": 2, "angle": 120, "time": 500}}
        ]
        """,
        ),
    ],
)
def test_translate_prompt_to_sequence(prompt, expected_response):
    with patch.dict(os.environ, {"OPENAI_API_KEY": "your_mocked_api_key"}):
        # Mock the openai.OpenAI class
        with patch("openai.OpenAI") as MockOpenAI:
            mock_openai_instance = MockOpenAI.return_value
            # Set up the mock for chat.completions.create method
            mock_openai_instance.chat.completions.create.return_value = ChatCompletion(
                id="chatcmpl-123",
                created=1677652288,
                model="gpt-3.5-turbo-1106",
                object="chat.completion",
                choices=[
                    Choice(
                        message=ChatCompletionMessage(
                            role="assistant", content=expected_response
                        ),
                        finish_reason="stop",
                        index=0,
                        text="",
                        logprobs={"content": []},
                    )
                ],
            )

            # Initialize the generator with the real API document path
            api_document_path = "~/source/ml-prototype/data/knowledge/robot_arm.md"
            generator = OperationSequenceGenerator(
                api_document_path=api_document_path, gpt_client=mock_openai_instance
            )

            # Test the method with a prompt
            result_json = generator.translate_prompt_to_sequence(prompt)

            # Assert that the response is as expected
            assert json.loads(result_json) == json.loads(
                expected_response
            ), "The response does not match the mocked GPT response."

            # Assert that the OpenAI API was called with the expected arguments
            mock_openai_instance.chat.completions.create.assert_called_once()


# Test for RobotArmControl's validation logic
def test_robot_arm_control_validation():
    robot_arm = SimulatedRobotArmControl()
    valid_sequence = (
        '[{"operation": "set_rgb_light", "parameters": {"R": 255, "G": 0, "B": 0}}]'
    )
    robot_arm.execute_operations(valid_sequence)  # Should pass without exception

    invalid_sequence = "not a json"
    with pytest.raises(ValueError):
        robot_arm.execute_operations(invalid_sequence)

    invalid_sequence_format = '[{"wrong_key": "value"}]'
    with pytest.raises(ValueError):
        robot_arm.execute_operations(invalid_sequence_format)


# @pytest.mark.parametrize(
#     "operations_json, expected_output",
#     [
#         (
#             """[{"operation": "set_rgb_light", "parameters": {"R": 255, "G": 0, "B": 0}}]""",
#             "Executing set_rgb_light with parameters: {'R': 255, 'G': 0, 'B': 0}",
#         ),
#         (
#             """[{
#                 "operation": "move_single_servo",
#                 "parameters": {"id": 1, "angle": 90, "time": 500}
#             }]""",
#             "Executing move_single_servo with parameters: {'id': 1, 'angle': 90, 'time': 500}",
#         ),
#         (
#             """[
#                 {"operation": "set_rgb_light", "parameters": {"R": 255, "G": 0, "B": 0}},
#                 {"operation": "move_single_servo", "parameters": {"id": 1, "angle": 90, "time": 500}}
#             ]""",
#             "Executing set_rgb_light with parameters: {'R': 255, 'G': 0, 'B': 0}\n"
#             "Executing move_single_servo with parameters: {'id': 1, 'angle': 90, 'time': 500}",
#         ),
#     ],
# )
# def test_simulate_multiple_operations(capsys, operations_json, expected_output):
#     simulator = SimulatedRobotArmControl()
#     # Call execute_operations with JSON string of operations
#     simulator.execute_operations(operations_json)

#     # Capture the printed output
#     captured_output = capsys.readouterr().out.strip()
#     assert (
#         captured_output == expected_output
#     ), f"Expected: {expected_output}, got: {captured_output}"
