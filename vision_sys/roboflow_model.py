# 1. Import the library
from inference_sdk import InferenceHTTPClient, InferenceConfiguration

# 2. Connect to your local server
client = InferenceHTTPClient(
    api_url="http://localhost:9001", # Local server address
    api_key="DQkgoUzTWeoKojYeOT9X"
).configure(InferenceConfiguration(
    api_key_transport="header"  # header-based auth (inference server v1.5.0+)
))

# 3. Run your workflow on an image
result = client.run_workflow(
    workspace_name="eyes-on-inanimate-objects",
    workflow_id="x-arm-vision-1-rfdetr-seg-small-t1-vx-arm-vision-1-rfdetr-seg-small-t1-logic",
    images={
        "image": "YOUR_IMAGE.jpg" # Path to your image file
    },
    use_cache=True # Speeds up repeated requests
)

# 4. Get your results
print(result)

