from openai import AzureOpenAI

from riberry.credentials import load_azure_credentials


class Chatbot:
    def __init__(self, system_prompt="You are a helpful assistant.", model="gpt-4.1-nano"):
        api_key, endpoint = load_azure_credentials("/etc/opt/riberry/credentials.json")
        self.client = AzureOpenAI(
            api_version="2024-12-01-preview",
            azure_endpoint=endpoint,
            api_key=api_key,
        )
        self.set_system_prompt(system_prompt)
        self.model = model

    def set_system_prompt(self, system_prompt):
        self.system_prompt = system_prompt

    def generate_response(self, user_prompt):
        try:
            response = self.client.chat.completions.create(
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                max_completion_tokens=800,
                temperature=1.0,
                top_p=1.0,
                frequency_penalty=0.0,
                presence_penalty=0.0,
                model=self.model
            )
            return response.choices[0].message.content
        except Exception as e:
            return f"Error: {e}"
