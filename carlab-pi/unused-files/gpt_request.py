from openai import OpenAI

context_desc = "Categorize the following commands into one of these five buckets. \
1: Go forward (F). \
2: Go backwards (B). \
3: Turn left (L). \
4: Turn right (R). \
5: Stop, no movement (S). \
If unsure, default to Stop (S). \
Use either one or two commands in order to accomplish any task. \
Give me just a series of letters, for example 'FB'. Don't give any additional text."

client = OpenAI()

assistant = client.beta.assistants.create(
  name="Pathfinder",
  description=context_desc,
  model="gpt-3.5-turbo",
)

thread = client.beta.threads.create(
  messages=[
    {
      "role": "user",
      "content": "Create 3 data visualizations based on the trends in this file.",
      "file_ids": [file.id]
    }
  ]
)


