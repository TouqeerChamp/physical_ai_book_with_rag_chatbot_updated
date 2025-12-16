const path = require('path');
const fs = require('fs');
const express = require('express');
const cors = require('cors');
const axios = require('axios'); // Import axios for health check
require('dotenv').config();
const { GoogleGenAI } = require('@google/genai');
const { QdrantClient } = require('@qdrant/js-client-rest');

const ai = new GoogleGenAI({ apiKey: process.env.GEMINI_API_KEY });

const COLLECTION_NAME = 'humanoid_robotics_textbook';
const EMBEDDING_MODEL = 'text-embedding-004'; // Sasta aur kafi accha embedding model
let qdrantClient; // Declare qdrantClient at module level to be accessible globally

// Text ko chote tukdon (chunks) mein taqseem karne ke liye
function splitTextIntoChunks(text, chunkSize = 1000, overlap = 200) {
    const chunks = [];
    let i = 0;
    while (i < text.length) {
        let end = Math.min(i + chunkSize, text.length);
        if (end < text.length) {
            let lastSpace = text.lastIndexOf(' ', end);
            if (lastSpace > i) {
                end = lastSpace;
            }
        }
        chunks.push(text.substring(i, end));
        i = end - overlap;
    }
    return chunks;
}

// Ingestion ka asal function
async function ingestData() {
    console.log('--- Starting Data Ingestion Process ---');

    // Naya Health Check test yahan add karein
    try {
        const healthCheck = await axios.get('http://172.17.0.1:6336/health');
        console.log(`Qdrant Health Status: ${healthCheck.status}`);
        if (healthCheck.status !== 200) {
          console.error("Qdrant health check failed. Stopping ingestion.");
          return;
        }
      } catch (error) {
        console.error("Qdrant connection FAILED (Health Check). Please ensure Docker is running and port 6336 is open.");
        return;
      }

    // QdrantClient ko yahan initialize karein, health check ke baad hi
    qdrantClient = new QdrantClient({
        host: '172.17.0.1',
        port: 6336,
        // Connection settings:
        timeout: 5000, // 5 seconds timeout
        https: false,
    });

    try {
        const filePath = path.join(__dirname, 'data', 'chapter_text.txt');

        // Ensure data directory exists (waise Agent ne pehle hi bana diya hai)
        const dataDir = path.join(__dirname, 'data');
        if (!fs.existsSync(dataDir)) {
            fs.mkdirSync(dataDir);
        }

        if (!fs.existsSync(filePath)) {
             console.error(`Error: Data file not found at ${filePath}. Please ensure chapter_text.txt is in the data folder.`);
             return;
        }

        const fileContent = fs.readFileSync(filePath, 'utf8');

        // 1. Text ko chunks mein baant dein
        const chunks = splitTextIntoChunks(fileContent);
        console.log(`Split file into ${chunks.length} chunks.`);

        // 2. Qdrant Collection check karein aur banaen (agar zaroori ho)
        const collections = await qdrantClient.getCollections();
        const collectionExists = collections.collections.some(
            (c) => c.name === COLLECTION_NAME
        );

        if (!collectionExists) {
            console.log(`Creating collection: ${COLLECTION_NAME}`);
            // Gemini Embedding model (text-embedding-004) ki vector size 768 hoti hai
            await qdrantClient.createCollection(COLLECTION_NAME, {
                vectors: { size: 768, distance: 'Cosine' },
            });
            console.log('Collection created successfully.');
        } else {
            console.log('Collection exists. Deleting existing points for a clean ingest.');
            await qdrantClient.delete(COLLECTION_NAME, {
                filter: {},
                points: [{all: true}],
                wait: true,
            });
        }

        // 3. Embeddings banaen aur Qdrant mein points daalen
        const points = [];
        for (let i = 0; i < chunks.length; i++) {
            const chunk = chunks[i];

            // Gemini Embedding API call
            const embeddingResult = await ai.embed.batchEmbedContents({
                model: EMBEDDING_MODEL,
                requests: [{ content: chunk }],
            });

            const vector = embeddingResult.embeddings[0].values;

            points.push({
                id: i + 1,
                vector: vector,
                payload: {
                    text: chunk,
                    chapter: '1. Introduction',
                    chunk_index: i,
                },
            });

            console.log(`Generated embedding for chunk ${i}`);
        }

        // 4. Batch mein points ko Qdrant mein daal dein
        await qdrantClient.upsert(COLLECTION_NAME, {
            wait: true,
            points: points,
        });

        console.log(`--- Data Ingestion Complete! Added ${points.length} points to Qdrant ---`);

    } catch (error) {
        console.error('Data Ingestion Failed:', error.message);
        console.error('Check 1: Is Qdrant (http://localhost:6336) running in Docker?');
        console.error('Check 2: Is your GEMINI_API_KEY valid in the .env file?');
    }
}

// Use a specific collection name for the document storage API endpoints
const DOCUMENT_COLLECTION_NAME = 'documents';

const app = express();
const PORT = process.env.PORT || 3003;

// Middleware
app.use(cors({
  origin: '*',
  credentials: true,
  optionsSuccessStatus: 200
}));
app.use(express.json());

// Health check endpoint
app.get('/health', (req, res) => {
  res.status(200).json({ status: 'OK', message: 'Server is running' });
});

// Endpoint to store documents in Qdrant
app.post('/api/documents', async (req, res) => {
  try {
    const { documents } = req.body; // Array of documents with content and metadata

    if (!Array.isArray(documents)) {
      return res.status(400).json({ error: 'Documents must be an array' });
    }

    // Ensure collection exists using the document-specific collection name
    await qdrantClient.recreateCollection(DOCUMENT_COLLECTION_NAME, {
      vectors: {
        size: 1536, // Assuming OpenAI embedding size, adjust as needed
        distance: 'Cosine',
      },
    });

    // Prepare points for insertion
    const points = documents.map((doc, index) => ({
      id: Date.now() + index, // Simple ID generation, in production use UUID
      vector: doc.embedding || [], // Embedding vector
      payload: {
        content: doc.content,
        metadata: doc.metadata || {},
      },
    }));

    // Store documents in Qdrant
    await qdrantClient.upsert(DOCUMENT_COLLECTION_NAME, {
      wait_result: true,
      points,
    });

    res.status(200).json({ message: 'Documents stored successfully', count: documents.length });
  } catch (error) {
    console.error('Error storing documents:', error);
    res.status(500).json({ error: 'Failed to store documents' });
  }
});

// Endpoint to search documents in Qdrant
app.post('/api/search', async (req, res) => {
  try {
    const { query, topK = 5 } = req.body;

    if (!query) {
      return res.status(400).json({ error: 'Query is required' });
    }

    // Perform search in Qdrant using the document-specific collection name
    const results = await qdrantClient.search(DOCUMENT_COLLECTION_NAME, {
      vector: query, // This should ideally be the embedding of the query
      limit: topK,
      with_payload: true,
    });

    res.status(200).json({ results });
  } catch (error) {
    console.error('Error searching documents:', error);
    res.status(500).json({ error: 'Failed to search documents' });
  }
});

// Endpoint to chat with the RAG system (will use the textbook collection)
app.post('/api/chat', async (req, res) => {
    const { query } = req.body;
    console.log(`Received query for RAG: ${query}`);

    try {
        // Generate embedding for the query using the Gemini API
        const embeddingResult = await ai.embed.batchEmbedContents({
            model: EMBEDDING_MODEL,
            requests: [{ content: query }],
        });

        const queryVector = embeddingResult.embeddings[0].values;

        // Search in the textbook collection for relevant context
        const searchResults = await qdrantClient.search(COLLECTION_NAME, {
            vector: queryVector,
            limit: 3, // Get top 3 most relevant chunks
            with_payload: true,
        });

        // Format the context from search results
        const context = searchResults.map(result => result.payload.text).join('\n\n');

        // Construct the prompt for the Gemini model
        const fullPrompt = `Context information is below:\n\n${context}\n\nUsing only the context information provided above, please answer the following question: ${query}`;

        // Use the Gemini API to generate a response based on the retrieved context
        const model = ai.getGenerativeModel({ model: "gemini-pro" });
        const result = await model.generateContent(fullPrompt);
        const response = await result.response.text();

        res.json({
            response: response
        });
    } catch (error) {
        console.error('Error processing chat query:', error);
        res.status(500).json({ error: 'Failed to process chat query' });
    }
});

// Server start hone se pehle data ingest karein
ingestData();

// Start the server
app.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}`);
  console.log(`Health check: http://localhost:${PORT}/health`);
});