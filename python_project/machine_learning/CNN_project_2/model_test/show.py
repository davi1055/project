# sklearn plt sns
from sklearn.metrics import (
    confusion_matrix,
    accuracy_score,
    precision_score,
    recall_score,
    f1_score,
)
import matplotlib.pyplot as plt
import seaborn as sns

# custom
from logger.logger import log


def showConfusionMatrix(true_labels, predicted_labels):
    cm = confusion_matrix(true_labels, predicted_labels)
    accuracy = accuracy_score(true_labels, predicted_labels)
    precision = precision_score(true_labels, predicted_labels, average="weighted")
    recall = recall_score(true_labels, predicted_labels, average="weighted")
    f1 = f1_score(true_labels, predicted_labels, average="weighted")
    log(cm, "DEBUG")
    log(f"Accuracy: {accuracy:.2f}", "DEBUG")
    log(f"Precision: {precision:.2f}", "DEBUG")
    log(f"Recall: {recall:.2f}", "DEBUG")
    log(f"F1 Score: {f1:.2f}", "DEBUG")

    plt.figure(figsize=(10, 10))
    sns.heatmap(
        cm,
        annot=True,
        fmt="d",
        cmap="Blues",
        xticklabels=["0", "1", "2", "3", "4", "5", "6", "7", "8", "9"],
        yticklabels=["0", "1", "2", "3", "4", "5", "6", "7", "8", "9"]
    )
    plt.xlabel("Predicted")
    plt.ylabel("True")
    plt.title("Confusion Matrix")
    plt.show()
