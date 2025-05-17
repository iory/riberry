#!/usr/bin/env python3

from riberry.chatbot import Chatbot


def main():
    system_prompt = "あなたはユーザーの入力から操作対象物体の名前を英語で抽出して返すアシスタントです。物体名だけを英語で答えてください。"
    model = "gpt-4.1-nano"
    chatbot = Chatbot(system_prompt=system_prompt, model=model)

    print(f"Chatbot CLI ({model}) - Type 'exit' to quit\n")
    print(f"System prompt: {system_prompt}")
    while True:
        # Example: "今からコーヒーポットの使い方を教えます。"
        user_prompt = input("You: ")
        if user_prompt.lower() in ['exit', 'quit']:
            print("Goodbye!")
            break

        system_output = chatbot.generate_response(user_prompt)
        print(f"Bot: {system_output}\n")


if __name__ == '__main__':
    main()
