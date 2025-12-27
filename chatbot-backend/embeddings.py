from sentence_transformers import SentenceTransformer
import torch

class EmbeddingEngine:
    def __init__(self, model_name='all-MiniLM-L6-v2'):
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = SentenceTransformer(model_name, device=self.device)
        print(f"Embedding Engine initialized on {self.device}")

    def generate(self, text):
        if isinstance(text, str):
            text = [text]
        embeddings = self.model.encode(text)
        return embeddings.tolist()[0] if len(embeddings) == 1 else embeddings.tolist()
