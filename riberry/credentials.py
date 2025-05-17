import json
import os


def load_azure_credentials(filepath):
    """Load credentials and export them as environment variable"""
    if not os.path.exists(filepath):
        return
    print(f"Load credentials from {filepath}")
    try:
        with open(filepath) as f:
            config = json.load(f)
        api_key = config.get('AZURE_API_KEY')
        endpoint = config.get('AZURE_ENDPOINT')
        if api_key:
            os.environ['AZURE_API_KEY'] = api_key
        if endpoint:
            os.environ['AZURE_ENDPOINT'] = endpoint
    except Exception as e:
        print(f"Error reading config file: {e}")
        return None
    # Check environment variables even without credential json
    try:
        api_key = os.environ['AZURE_API_KEY']
        endpoint = os.environ['AZURE_ENDPOINT']
    except KeyError as e:
        print(f"[WARNING] Environment variable '{e}' is not set.")
        print("Azure API cannot be used.\n")
        api_key = ""
        endpoint = ""
    return (api_key, endpoint)
