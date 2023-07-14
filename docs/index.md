---
title: "CodeBotler + RoboEval"
subtitle: Deploying and Evaluating LLMs to Program Service Mobile Robots
authors: [Francesca Lucchetti, Zichao Hu, Anders Freeman, Sadanand Modak, Yash Saxena, Luisa Mao, Claire Schlesinger, Arjun Guha, Joydeep Biswas]
layout: project
order: 1
---

# Abstract
Recent advancements in the capabilities of large language models (LLMs) have spurred interest in using them for generating robot task programs from natural language task descriptions. Early results indicate that LLMs are indeed capable of generating reasonable robot programs. However, such initial promise has been tempered by results showing that 1) LLMs frequently generate imprecise results that are not executable, and 2) large-scale testing and evaluation has been hindered by the lack of existing benchmarks. We contribute **CodeBotler** and **RoboEval** to address the two issues respectively. CodeBotler is an open-source tool to generate general-purpose service robot programs from natural language, and to deploy such programs on general-purpose autonomous mobile robots. The RoboEval benchmark consists of multiple service robot tasks with multiple prompts per task, multiple prompt variations per task, and temporal checks to test generated programs for correctness. 

# Overview

**CodeBotler** is an open-source tool to generate general-purpose service robot programs from natural language, and to deploy such programs on general-purpose autonomous mobile robots. It performs program generation with an _embedded domain-specific language_ (eDSL) in a programming
language that LLMs are already adept in: Python. By abstracting robot
skills in the eDSL, we release CodeBotler, a robot-agnostic deployment
system for executing generated programs on any general-purpose mobile robot. By
embedding the eDSL in Python, CodeBotler drastically reduces the number of
syntax and run-time errors of generated code. 

**RoboEval** is a code completion benchmark that test for both correctness and robustness of LLM-generated robot programs. This proposed benchmark 1) evaluates the *execution traces* of programs; 2) checks whether the execution traces satisfy *temporal logic* properties that encode correctness for each task; and 3) *varies the prompts* for each task to test for robustness of generated programs.

<div style="justify-content: center; align-items: center; display: flex;">
<img src="assets/images/RoboEvalFig1.jpg" style="width:100%; max-width:500px; height:auto;"/>
</div>

<div style="justify-content: center; align-items: center; display: flex;">
CodeBotler converts a) natural language task prompts to b) python programs leveraging robot skill abstraction, and c) executes the programs on a real robot. d) RoboEval verifies results via LTL checking of simulation traces.
</div>



# Preliminary RoboEval Results
<div>
We have released benchmarks on 5 different tasks:
- **HalloweenList**: Go to every office, and if there is anyone there, ask if they'd like a chocolate, caramel, or gummy. Come back and tell me how many of each we need to buy.
- **LunchBreak**: Ask if Alice and Bob in their offices are up for lunch. If yes, tell them that we'll meet in the lobby in 5 minutes. Come back and tell me who all are joining for lunch.
- **StaplerSupply**: Check every printer room for a stapler, and come back and tell me which ones do not have a stapler.
- **MovieMessenger**: Ask Sally in her office if she wants to go to the cinema with Mark. Go to Mark's office and tell him Sally's answer. If Sally says yes, ask Mark whether he wants to leave at 4PM, 5PM, or 6PM - then go tell Sally what time Mark is leaving.
- **ElevatorTour**: Go to the elevator. Wait until someone shows up and ask them if they are here for the tour. If yes, welcome them to the university, ask them to follow you, and take them to the main conference room. If not, wait for the next person. When you get to the conference room, say you have arrived at the conference room and also say enjoy your visit here!


<div style="justify-content: center; align-items: center; display: flex;">
<img src="assets/images/results_v3.png" style="width:100%; max-width:500px; height:auto;"/>
</div>
<div style="justify-content: center; align-items: center; display: flex;">
pass@1 results for PaLM (`text-bison-001`), StarCoder, and GPT3.5 (`text-davinci-003`) - errorbars show variations resulting from rephrasing task prompts.
</div>

# CodeBotler Demo