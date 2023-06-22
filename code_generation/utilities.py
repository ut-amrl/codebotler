#!/usr/bin/env python3
import os
import sys
import roslib
import re


def add_pythonpath_load_amrl_msgs_cd_rel(add_rel_path, cd_rel_path):
    sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), add_rel_path))
    os.chdir(os.path.join(os.path.dirname(os.path.realpath(__file__)), cd_rel_path))
    roslib.load_manifest('amrl_msgs')


def add_pythonpath(add_rel_path):
    sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), add_rel_path))


def cd_rel(cd_rel_path):
    os.chdir(os.path.join(os.path.dirname(os.path.realpath(__file__)), cd_rel_path))


def load_amrl_msgs():
    roslib.load_manifest('amrl_msgs')


def process_command_string(command_string, DSL_FNS):
    """
    1. Check if last function call is say, return a boolean
    2. For go_to, remove "'s" from the end of words, etc and return new string
    3. Prepend self. to each function call and return new string
    """

    def fn1(input_str):
        # The regular expression pattern for a function name
        pattern = r'\b([a-zA-Z_][a-zA-Z_0-9]*)\s*\('
        # Find all matches in the command_string
        all_matches = re.findall(pattern, input_str)
        # Filter matches based on DSL
        matches = [match for match in all_matches if match in DSL_FNS]
        return matches[-1] == "say"

    def fn2(input_str):
        def postprocess(inp):
            inp = inp.replace("'s", "")
            # Remove punctuation
            inp = ''.join(ch for ch in inp if ch.isalnum() or ch.isspace())
            # Split the string into words
            words = inp.split()
            # Lower case for all words
            words = [word.lower() for i, word in enumerate(words)]
            # Join the words back into a string
            output_str = ' '.join(words)
            return output_str

        # Apply postprocess to argument of go_to function if it's a string
        def replacer(match):
            func_name = match.group(1)
            arg = match.group(2)
            if func_name == "go_to" and arg.startswith("\"") and arg.endswith("\""):
                # Strip quotes, postprocess the argument, and then add quotes back
                return 'go_to(\"{}\")'.format(postprocess(arg.strip("\"")))
            else:
                return match.group(0)

        return re.sub(r'(\b[A-Za-z_][A-Za-z0-9_]*\b)\((.*?)\)', replacer, input_str)

    def fn3(input_str):
        return re.sub(r'\b([A-Za-z_][A-Za-z0-9_]*\b)\(', 
                    lambda m: "self." + m.group() if m.group(1) in DSL_FNS else m.group(), 
                    input_str)

    return fn3(fn2(command_string)), fn1(command_string)
