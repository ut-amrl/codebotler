---
title: "CodeBotler:"
subtitle: Deploying and Evaluating Language Models for Robot Program Generation
authors: [Francesca Lucchetti, Zichao Hu, Sadanand Modak, Yash Saxena, Anders Freeman, Luisa Mao, Claire Schlesinger, Arjun Guha, Joydeep Biswas]
layout: project
order: 1
---

# Abstract
Recent advancements in the capabilities of large language
models (LLMs) have spurred interest in using them for gener-
ating robot task policies from natural language task descrip-
tions. Early results indicate that LLMs are indeed ca-
pable of generating reasonable general-purpose service robot
programs. However, such initial promise has been tempered
by results showing that 1) LLMs frequently generate impre-
cise results that are not executable, and 2) large-scale testing
and evaluation has been hindered by the lack of existing
benchmarks. We contribute CODEBOTLER and ROBOEVAL
to address the two issues respectively.

# Overview
<div>
<img src="assets/images/RoboEvalFig1.jpg" style="width:100%"/>
</div>
CodeBotler is an open-source tool to generate general-
purpose service robot programs from natural language, and
to deploy such programs on general-purpose autonomous
mobile robots. 

RoboEval is a code completion benchmark that test for both correctness and robustness of LLM-generated robot programs. This proposed benchmark evaluates the execution
traces of programs in simulated environments, heck
whether the execution traces satisfy temporal logic properties, vary the
prompts and evaluate the resulting programs to evaluate for
robustness



a) CODEBOTLER converts a) natural language task prompts to
b) python programs and c) executes on a real robot. d) ROBOEVAL
verifies results via LTL checking of simulation traces.


# Preliminary RoboEval Results
<div class="row">
<div class="col-md-6 text-center">
<img src="assets/images/results_v3.png" style="width:100%"/>
</div>
<div class="col-md-6">
We have released benchmarks on 5 different tasks:
- HalloweenList: 
- LunchBreak: Ask if Alice and Bob in their offices are up for lunch. If yes, tell them that we'll meet in the lobby in 5 minutes. Come back and tell me who all are joining for lunch.
- StaplerSupply:
- MovieMessenger:
- ElevatorTour:

</div>
</div>

# CodeBotler Demo