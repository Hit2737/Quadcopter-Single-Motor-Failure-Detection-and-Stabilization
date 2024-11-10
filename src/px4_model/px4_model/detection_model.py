import os
import time
import torch
import torch.nn as nn
import torch.optim as optim
from torch.nn.utils.rnn import pack_padded_sequence, pad_packed_sequence

class MotorFailureDetectionModel(nn.Module):
    def __init__(self, input_size, hidden_size=64, num_layers=3, output_size=1, dropout_prob=0.3):
        super(MotorFailureDetectionModel, self).__init__()
        
        # LSTM layers
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers, batch_first=True, dropout=dropout_prob)
        
        # Fully connected layer
        self.fc = nn.Linear(hidden_size, output_size)
        
        # Dropout layer
        self.dropout = nn.Dropout(dropout_prob)
        
        # Activation function
        self.sigmoid = nn.Sigmoid()

    def forward(self, x, lengths):
        # Pack the padded sequence
        packed_input = pack_padded_sequence(x, lengths, batch_first=True, enforce_sorted=False)
        
        # LSTM layers
        packed_output, _ = self.lstm(packed_input)
        
        # Unpack the sequence
        lstm_out, _ = pad_packed_sequence(packed_output, batch_first=True)
        
        # Fully connected layer with dropout and activation
        out = self.fc(lstm_out)
        out = self.sigmoid(out)
        
        return out

    def train_model(self, train_loader, num_epochs=10, learning_rate=0.001, device='cpu'):
        # Set model to training mode
        self.train()
        
        # Loss and optimizer
        criterion = nn.BCELoss()
        optimizer = optim.Adam(self.parameters(), lr=learning_rate)

        # Move model to device
        self.to(device)
        
        for epoch in range(num_epochs):
            epoch_loss = 0.0
            start_time = time.time()
            for batch_idx, (data, target, lengths) in enumerate(train_loader):
                # Move data and target to device
                data, target, lengths = data.to(device), target.to(device), lengths.to(device)
                
                # Zero the gradients
                optimizer.zero_grad()
                
                # Forward pass
                output = self(data, lengths)
                
                # Calculate loss
                # Mask the padded values in the target
                mask = torch.arange(target.size(1)).expand(len(lengths), target.size(1)).to(device) < lengths.unsqueeze(1)
                masked_output = output[mask]
                masked_target = target[mask]
                loss = criterion(masked_output, masked_target)
                
                # Backward pass and optimization
                loss.backward()
                optimizer.step()
                
                # Accumulate loss for the epoch
                epoch_loss += loss.item()
                
                # Print progress every 100 batches
                if batch_idx % 5 == 0 and (((epoch+1) % 10 == 0) or (epoch == 0)):
                    print(f'Epoch [{epoch+1}/{num_epochs}] | Batch [{batch_idx+1}/{len(train_loader)}] | Loss: {loss.item():.4f}\n')

            # Print average loss for the epoch
            if ((epoch+1) % 10 == 0) or (epoch == 0):
                print(f'{"-"*70}\n')
                print(f'Epoch [{epoch+1}/{num_epochs}] | Average Loss: {epoch_loss/len(train_loader):.4f} | Time: {time.time()-start_time:.2f}s')
                print(f'\n{"-"*70}\n')

    def save_model(self, file_name):
        # Save the model's state dict
        directory = "../models"
        file_path = os.path.join(directory, file_name)
        os.makedirs(directory, exist_ok=True)
        torch.save(self.state_dict(), file_path)
        print(f'Model saved to {file_path}')
    
    def load_model(self, file_path, device='cpu'):
        # Load the model's state dict
        self.load_state_dict(torch.load(file_path, map_location=device))
        self.to(device)