# 1. Import the library
from inference_sdk import InferenceHTTPClient, InferenceConfiguration

# 2. Connect to your workflow
client = InferenceHTTPClient(
    api_url="https://serverless.roboflow.com",
    api_key="DQkgoUzTWeoKojYeOT9X"
).configure(InferenceConfiguration(
    api_key_transport="header"  # header-based auth (inference v1.5.0+)
))

# 3. Run your workflow on an image
result = client.run_workflow(
    workspace_name="eyes-on-inanimate-objects",
    workflow_id="xarmvision-vxarmvision-anv01-1-rfdetr-small-t1-logic",
    images={
        "image": "C:\\Users\\mcdon\\Downloads\\lab_equip\\photo1_Color.png" # Path to your image file
    },
    use_cache=True # Speeds up repeated requests
)

# 4. Get your results
print(result)
