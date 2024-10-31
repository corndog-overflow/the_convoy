import matplotlib.pyplot as plt
import tensorflow as tf

# Load your dataset (as you've already done)
dataset = tf.keras.utils.image_dataset_from_directory(
    'training',
    labels='inferred',
    label_mode='categorical',
    batch_size=32,
    image_size=(224, 224)
)

# Function to display sample images
def display_sample_images(dataset):
    # Get a batch of images and labels
    for images, labels in dataset.take(1):  # Take the first batch
        break

    # Set the number of images to display
    num_images = min(9, images.shape[0])  # Display up to 9 images
    plt.figure(figsize=(10, 10))
    
    for i in range(num_images):
        plt.subplot(3, 3, i + 1)  # Create a 3x3 grid
        plt.imshow(images[i].numpy().astype("uint8"))  # Convert to uint8 for display
        plt.title(f"Label: {labels[i].numpy().argmax()}")  # Display the index of the class
        plt.axis("off")  # Hide axes

    plt.show()

# Call the function to display images
display_sample_images(dataset)
