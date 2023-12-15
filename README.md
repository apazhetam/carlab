# ECE302 Carlab

## Ashwin Pazhetam and Abhi Vellore

# Objective:
To use Large Language Models (LLM) to control the mini car’s motion. Incorporate a speech-to-text module to feed into the Open AI GPT-4.0 API, performed on a Raspberry Pi 3. The LLM will output a series of basic directional commands, that is then sent via UART communication to the PSOC. The PSOC then interprets the commands, and by using a state machine, determines which action to perform.

Our systems are split into a PSOC board and a Raspberry Pi. Code is separated accordingly.
Please reference final report for design choices and implementation details.