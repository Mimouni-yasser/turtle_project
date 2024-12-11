import torch
import torchvision
from torchvision.models import resnet34, ResNet34_Weights
from PIL import Image
import cv2
import numpy as np

model = resnet34()
state_dict = torch.load(f= '/home/user/ros2_humble/src/capture_images/capture_images/08_resnet34_data_10_percent_10_epochs.pth',weights_only=False, map_location=torch.device('cpu'))

filtered_state_dict = {k: v for k, v in state_dict.items() if "classifier.1" not in k}

model.load_state_dict(filtered_state_dict)

def crop_largest_purple_rectangle(frame):
    """
    Detects the largest purple rectangle in the frame and crops the image to only include that rectangle.
    
    :param frame: The input image frame from cv2 (e.g., from a video stream).
    :return: Cropped image containing only the largest purple rectangle, or the original frame if none is found.
    """
    # Convert the frame to HSV color space for easier color detection
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define the range for the color purple in HSV
    lower_purple = np.array([120, 50, 50])  # Adjust the values as needed
    upper_purple = np.array([160, 255, 255])  # Adjust the values as needed
    
    # Create a mask to isolate purple regions in the frame
    purple_mask = cv2.inRange(hsv_frame, lower_purple, upper_purple)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(purple_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        print("No purple regions detected.")
        return frame  # Return the original frame if no contours are found
    
    # Find the largest contour by area
    largest_contour = max(contours, key=cv2.contourArea)
    
    # Approximate the contour to a polygon and check if it's a rectangle
    epsilon = 0.02 * cv2.arcLength(largest_contour, True)
    approx = cv2.approxPolyDP(largest_contour, epsilon, True)
    
    if len(approx) == 4:  # Check if the shape is a quadrilateral
        # Get the bounding box of the largest rectangle
        x, y, w, h = cv2.boundingRect(approx)
        # Crop the frame to the bounding box
        cropped_image = frame[y:y+h, x:x+w]
        return cropped_image
    else:
        print("No rectangular shapes detected in purple regions.")
        return frame  # Return the original frame if no rectangle is detected


def predict_image_label(image):
    # Preprocess the image for the model
    preprocess = torchvision.transforms.Compose([
        torchvision.transforms.Resize(256),
        torchvision.transforms.CenterCrop(224),
        torchvision.transforms.ToTensor(),
        torchvision.transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])
    input_tensor = preprocess(image)
    input_batch = input_tensor.unsqueeze(0)  # Create a mini-batch as expected by the model

    # Move the input and model to GPU for speed if available
    if torch.cuda.is_available():
        input_batch = input_batch.to('cuda')
        model.to('cuda')

    with torch.no_grad():
        output = model(input_batch)

    # Tensor of shape 1000, with confidence scores over Imagenet's 1000 classes
    probabilities = torch.nn.functional.softmax(output[0], dim=0)

    # Get the top 5 most likely labels
    top5_prob, top5_catid = torch.topk(probabilities, 5)
    for i in range(top5_prob.size(0)):
        print(f"{i+1}: {top5_catid[i].item()} ({top5_prob[i].item():.6f})")

img = Image.open('./20241113_113524.jpg')
predict_image_label(img)
# Open a connection to the camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    cropped = crop_largest_purple_rectangle(frame)

    # Convert the frame to a PIL image
    image = Image.fromarray(cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB))

    # Predict the label of the image
    predict_image_label(image)

    # Display the resulting frame
    cv2.imshow('Video Stream', cropped)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture and close windows
cap.release()
cv2.destroyAllWindows()