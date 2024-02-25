import argparse
import json
import os
from typing import Any, Dict, List

import faiss
import numpy as np
from dotenv import load_dotenv
from openai import OpenAI
from utils import Logger


class InstructionIndexer:

    def __init__(self):
        """
        Initialize the InstructionIndexer class.

        Args:
            json_file_path (str): Path to the JSON file containing instructions.
        """
        load_dotenv(override=True)
        self.logger = Logger(__file__)
        self.client = OpenAI()
        self.operation_sequences = []
        self.index = None

    def create_index(
        self, command_bank_file_path: str, index_destination: str, data_destination: str
    ) -> None:
        """
        Create the FAISS index for instructions.
        """
        instructions_data = self._load_json_file(command_bank_file_path)
        instructions = []
        operation_sequences = []
        for instruction, operations in instructions_data.items():
            instructions.append(instruction)
            operations_blob = {
                "instruction": instruction,
                "operations": operations["operations"],
            }
            operation_sequences.append(operations_blob)

        embeddings = self._embed_instructions(instructions)

        # Creating the FAISS index
        dimension = embeddings.shape[1]
        self.index = faiss.IndexFlatL2(dimension)
        self.index.add(embeddings)

        # Save the FAISS index and operation sequences
        self._save_index_and_data(
            operation_sequences=operation_sequences,
            index_destination=index_destination,
            data_destination=data_destination,
        )

    def _load_json_file(self, command_bank_file_path: str) -> Dict[str, Any]:
        """
        Load the JSON file containing instructions.

        Returns:
            dict: Dictionary containing instructions.
        """
        self.logger.info(f"Loading JSON file: {command_bank_file_path}")
        with open(command_bank_file_path, "r") as file:
            return json.load(file)

    def _embed_instructions(self, instructions: List[str]) -> np.ndarray:
        """
        Embed instructions using OpenAI's embedding API.

        Args:
            instructions (list): List of instructions.

        Returns:
            np.ndarray: Array of instruction embeddings.
        """
        self.logger.info(f"Embedding instructions...")
        embeddings = []
        for instruction in instructions:
            # Ensure instruction is a single line
            instruction = instruction.replace("\n", " ")
            # Create embedding
            response = self.client.embeddings.create(
                input=[instruction], model="text-embedding-3-small"
            )
            embeddings.append(response.data[0].embedding)
        return np.array(embeddings, dtype="float32")

    def _save_index_and_data(
        self,
        operation_sequences: List[Dict[str, Any]],
        index_destination: str,
        data_destination: str,
    ) -> None:
        """
        Save the FAISS index and operation sequences to files.

        Args:
            index_destination (str): Path to save the FAISS index.
            data_destination (str): Path to save the operation sequences.
        """
        # Ensure the index and data are created
        if self.index is None:
            raise ValueError("Index has not been created. Call create_index() first.")

        # Save the FAISS index
        if os.path.exists(index_destination):
            os.remove(index_destination)
        os.makedirs(os.path.dirname(index_destination), exist_ok=True)
        faiss.write_index(self.index, index_destination)

        # Save the operation sequences
        if os.path.exists(data_destination):
            os.remove(data_destination)
        os.makedirs(os.path.dirname(data_destination), exist_ok=True)
        with open(data_destination, "w") as file:
            json.dump(operation_sequences, file, indent=2)

    def load_index_and_data(
        self, index_path: str = None, data_path: str = None
    ) -> None:
        """
        Load the FAISS index and operation sequences from files.

        Args:
            index_path (str): Path to the FAISS index file.
            data_path (str): Path to the operation sequences file.
        """
        if not index_path:
            index_path = os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                "knowledge/index/instructions.index",
            )
        if not data_path:
            data_path = os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                "knowledge/index/instructions_data.json",
            )
        self.index = faiss.read_index(index_path)
        with open(data_path, "r") as file:
            self.operation_sequences = json.load(file)

    def retrieve_operation_sequences(self, instruction: str, k: int = 1) -> str:
        """
        Retrieve the operation sequences for a given query.

        Args:
            instruction (str): Query to search for.
            k (int): Number of operation sequences to retrieve.

        Returns:
            list: List of operation sequences.
        """
        # Embed the query
        query_embedding = self._embed_instructions([instruction])

        # Search the index
        _, indices = self.index.search(query_embedding, k)
        retrieved_operations = [self.operation_sequences[i] for i in indices[0]]
        json_blob = json.dumps(retrieved_operations[0])
        return json_blob


def parse_args():
    # Setup argument parser
    parser = argparse.ArgumentParser(description="Instruction Indexer")
    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    index_destination = os.path.join(
        os.path.dirname(os.path.dirname(__file__)), "knowledge/index/instructions.index"
    )

    data_destination = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        "knowledge/index/instructions_data.json",
    )

    # Subparser for creating index
    create_index_parser = subparsers.add_parser(
        "index", help="Create a new FAISS index from JSON data"
    )

    default_command_file_path = os.path.join(
        os.path.dirname(os.path.dirname(__file__)), "knowledge/command_bank.json"
    )
    create_index_parser.add_argument(
        "--command-bank-file-path",
        type=str,
        default=default_command_file_path,
        help="Path to the JSON file containing instructions",
    )

    create_index_parser.add_argument(
        "--index-destination",
        type=str,
        default=index_destination,
        required=False,
        help="Path to save the FAISS index",
    )

    create_index_parser.add_argument(
        "--data-destination",
        type=str,
        default=data_destination,
        required=False,
        help="Path to save the operation sequences",
    )

    # Subparser for querying index
    query_index_parser = subparsers.add_parser(
        "query", help="Query an existing FAISS index"
    )
    query_index_parser.add_argument(
        "-q", "--query", type=str, help="Query to search for"
    )
    query_index_parser.add_argument(
        "--index-path",
        type=str,
        default=index_destination,
        required=False,
        help="Path to the FAISS index file",
    )
    query_index_parser.add_argument(
        "--data-path",
        type=str,
        default=data_destination,
        required=False,
        help="Path to the JSON file containing operation sequences",
    )
    query_index_parser.add_argument(
        "--k",
        type=int,
        default=1,
        required=False,
        help="Number of operation sequences to retrieve",
    )

    # Parse arguments
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.command == "index":
        indexer = InstructionIndexer()
        indexer.create_index(
            command_bank_file_path=args.command_bank_file_path,
            index_destination=args.index_destination,
            data_destination=args.data_destination,
        )
        print("Index and data saved.")
    else:
        indexer = InstructionIndexer()
        indexer.load_index_and_data(
            index_path=args.index_path, data_path=args.data_path
        )
        operation_sequences = indexer.retrieve_operation_sequences(args.query, args.k)
        print(f"Operation sequences: {operation_sequences}")
