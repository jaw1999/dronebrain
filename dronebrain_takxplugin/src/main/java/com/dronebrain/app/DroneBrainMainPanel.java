package com.rivet.app;

import gov.takx.api.plugin.app.IAppPluginDelegate;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.swing.*;
import java.awt.*;
import java.lang.invoke.MethodHandles;

public class RivetMainPanel extends JPanel
{
    private static final Logger logger = LoggerFactory.getLogger(MethodHandles.lookup().lookupClass());

    private final IAppPluginDelegate delegate;
    private final RivetAPIClient apiClient;

    private JTextField ipField;
    private JButton connectButton;
    private JButton disconnectButton;
    private JButton executeButton;
    private JButton stopButton;
    private JButton killButton;
    private JLabel statusLabel;
    private JLabel currentStateLabel;
    private JTextArea outputArea;

    public RivetMainPanel(IAppPluginDelegate delegate)
    {
        this.delegate = delegate;
        this.apiClient = new RivetAPIClient();

        setLayout(new BorderLayout());
        setPreferredSize(new Dimension(400, 600));

        initializeComponents();
    }

    private void initializeComponents()
    {
        // Connection panel at the top
        JPanel connectionPanel = new JPanel(new GridBagLayout());
        connectionPanel.setBorder(BorderFactory.createTitledBorder("Connection"));
        GridBagConstraints gbc = new GridBagConstraints();
        gbc.insets = new Insets(5, 5, 5, 5);
        gbc.fill = GridBagConstraints.HORIZONTAL;

        // IP:Port field
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.weightx = 0.0;
        connectionPanel.add(new JLabel("IP:Port:"), gbc);

        gbc.gridx = 1;
        gbc.weightx = 1.0;
        ipField = new JTextField("localhost:5000");
        connectionPanel.add(ipField, gbc);

        // Connect/Disconnect buttons
        JPanel buttonPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
        connectButton = new JButton("Connect");
        connectButton.addActionListener(e -> connect());

        disconnectButton = new JButton("Disconnect");
        disconnectButton.setEnabled(false);
        disconnectButton.addActionListener(e -> disconnect());

        buttonPanel.add(connectButton);
        buttonPanel.add(disconnectButton);

        gbc.gridx = 0;
        gbc.gridy = 1;
        gbc.gridwidth = 2;
        connectionPanel.add(buttonPanel, gbc);

        add(connectionPanel, BorderLayout.NORTH);

        // Center panel with commands and state
        JPanel centerPanel = new JPanel(new BorderLayout());

        // Command buttons panel
        JPanel commandPanel = new JPanel(new GridBagLayout());
        commandPanel.setBorder(BorderFactory.createTitledBorder("Commands"));
        GridBagConstraints cmdGbc = new GridBagConstraints();
        cmdGbc.insets = new Insets(5, 5, 5, 5);
        cmdGbc.fill = GridBagConstraints.HORIZONTAL;
        cmdGbc.weightx = 1.0;

        executeButton = new JButton("Execute");
        executeButton.setEnabled(false);
        executeButton.addActionListener(e -> executeCommand());
        cmdGbc.gridx = 0;
        cmdGbc.gridy = 0;
        commandPanel.add(executeButton, cmdGbc);

        stopButton = new JButton("Stop");
        stopButton.setEnabled(false);
        stopButton.addActionListener(e -> stopCommand());
        cmdGbc.gridy = 1;
        commandPanel.add(stopButton, cmdGbc);

        killButton = new JButton("Kill");
        killButton.setEnabled(false);
        killButton.addActionListener(e -> killCommand());
        cmdGbc.gridy = 2;
        commandPanel.add(killButton, cmdGbc);

        centerPanel.add(commandPanel, BorderLayout.NORTH);

        // Current State panel - compact size
        JPanel statePanel = new JPanel(new BorderLayout());
        statePanel.setBorder(BorderFactory.createTitledBorder("Status"));
        currentStateLabel = new JLabel("Idle", SwingConstants.CENTER);
        currentStateLabel.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 11));
        currentStateLabel.setOpaque(true);
        currentStateLabel.setBackground(Color.LIGHT_GRAY);
        currentStateLabel.setForeground(Color.BLACK);
        statePanel.setPreferredSize(new Dimension(400, 45));
        statePanel.setMaximumSize(new Dimension(Integer.MAX_VALUE, 45));
        statePanel.add(currentStateLabel, BorderLayout.CENTER);

        centerPanel.add(statePanel, BorderLayout.SOUTH);

        add(centerPanel, BorderLayout.CENTER);

        // Output panel at bottom
        JPanel outputPanel = new JPanel(new BorderLayout());
        outputPanel.setBorder(BorderFactory.createTitledBorder("Output"));

        outputArea = new JTextArea(6, 30);
        outputArea.setEditable(false);
        outputArea.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 11));
        JScrollPane scrollPane = new JScrollPane(outputArea);

        outputPanel.add(scrollPane, BorderLayout.CENTER);

        add(outputPanel, BorderLayout.SOUTH);

        // Status bar
        JPanel statusBar = createStatusBar();
        add(statusBar, BorderLayout.PAGE_END);
    }

    private JPanel createStatusBar()
    {
        JPanel statusBar = new JPanel(new BorderLayout());
        statusBar.setBorder(BorderFactory.createEtchedBorder());

        statusLabel = new JLabel("Not connected");
        statusLabel.setForeground(Color.RED);
        statusBar.add(statusLabel, BorderLayout.WEST);

        return statusBar;
    }

    private void connect()
    {
        String ipPort = ipField.getText();

        if (ipPort == null || ipPort.trim().isEmpty())
        {
            JOptionPane.showMessageDialog(this, "IP:Port cannot be empty", "Error", JOptionPane.ERROR_MESSAGE);
            return;
        }

        connectButton.setEnabled(false);
        updateState("Connecting...", Color.ORANGE);
        appendOutput("Connecting to " + ipPort + "...");

        apiClient.testConnection(ipPort)
                .thenAccept(success -> SwingUtilities.invokeLater(() -> {
                    if (success)
                    {
                        try
                        {
                            apiClient.connect(ipPort);
                            updateUIState(true);
                            updateState("Connected - Idle", new Color(0, 128, 0));
                            appendOutput("✓ Successfully connected to " + apiClient.getServerUrl());
                        }
                        catch (Exception e)
                        {
                            logger.error("Failed to set connection", e);
                            updateState("Connection Failed", Color.RED);
                            appendOutput("✗ Connection failed: " + e.getMessage());
                            connectButton.setEnabled(true);
                        }
                    }
                    else
                    {
                        updateState("Connection Failed", Color.RED);
                        appendOutput("✗ Connection failed: Server not reachable");
                        JOptionPane.showMessageDialog(this, "Connection failed: Server not reachable at " + ipPort, "Connection Error", JOptionPane.ERROR_MESSAGE);
                        connectButton.setEnabled(true);
                    }
                }))
                .exceptionally(e -> {
                    SwingUtilities.invokeLater(() -> {
                        logger.error("Failed to connect", e);
                        updateState("Connection Failed", Color.RED);
                        appendOutput("✗ Connection failed: " + e.getMessage());
                        JOptionPane.showMessageDialog(this, "Connection failed: " + e.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
                        connectButton.setEnabled(true);
                    });
                    return null;
                });
    }

    private void disconnect()
    {
        apiClient.disconnect();
        updateUIState(false);
        updateState("Disconnected", Color.GRAY);
        appendOutput("Disconnected from server");
    }

    private void executeCommand()
    {
        updateState("EXECUTING", Color.ORANGE);
        appendOutput("Sending execute command...");

        apiClient.execute()
                .thenAccept(response -> SwingUtilities.invokeLater(() -> {
                    updateState("RUNNING", new Color(0, 128, 0));
                    appendOutput("✓ Execute started: " + response);
                }))
                .exceptionally(e -> {
                    SwingUtilities.invokeLater(() -> {
                        updateState("Execute Failed", Color.RED);
                        appendOutput("✗ Execute failed: " + e.getMessage());
                        JOptionPane.showMessageDialog(this, "Execute failed: " + e.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);

                        // Return to idle after 2 seconds
                        Timer timer = new Timer(2000, ev -> updateState("Connected - Idle", new Color(0, 128, 0)));
                        timer.setRepeats(false);
                        timer.start();
                    });
                    return null;
                });
    }

    private void stopCommand()
    {
        updateState("STOPPING", Color.ORANGE);
        appendOutput("Sending stop command...");

        apiClient.stop()
                .thenAccept(response -> SwingUtilities.invokeLater(() -> {
                    updateState("STOPPED", new Color(255, 165, 0)); // Orange
                    appendOutput("✓ Stop successful: " + response);
                }))
                .exceptionally(e -> {
                    SwingUtilities.invokeLater(() -> {
                        updateState("Stop Failed", Color.RED);
                        appendOutput("✗ Stop failed: " + e.getMessage());
                        JOptionPane.showMessageDialog(this, "Stop failed: " + e.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
                    });
                    return null;
                });
    }

    private void killCommand()
    {
        updateState("KILLING", Color.RED);
        appendOutput("Sending kill command...");

        apiClient.kill()
                .thenAccept(response -> SwingUtilities.invokeLater(() -> {
                    updateState("KILLED", Color.DARK_GRAY);
                    appendOutput("✓ Kill successful: " + response);
                }))
                .exceptionally(e -> {
                    SwingUtilities.invokeLater(() -> {
                        updateState("Kill Failed", Color.RED);
                        appendOutput("✗ Kill failed: " + e.getMessage());
                        JOptionPane.showMessageDialog(this, "Kill failed: " + e.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
                    });
                    return null;
                });
    }

    private void updateUIState(boolean connected)
    {
        connectButton.setEnabled(!connected);
        disconnectButton.setEnabled(connected);
        executeButton.setEnabled(connected);
        stopButton.setEnabled(connected);
        killButton.setEnabled(connected);
        ipField.setEnabled(!connected);

        if (connected)
        {
            statusLabel.setText("Connected to " + apiClient.getServerUrl());
            statusLabel.setForeground(new Color(0, 128, 0));
        }
        else
        {
            statusLabel.setText("Not connected");
            statusLabel.setForeground(Color.RED);
            updateState("Idle", Color.LIGHT_GRAY);
        }
    }

    private void updateState(String state, Color color)
    {
        SwingUtilities.invokeLater(() -> {
            currentStateLabel.setText(state.toUpperCase());
            currentStateLabel.setBackground(color);
            // Set text color based on background brightness
            if (color.equals(Color.LIGHT_GRAY) || color.equals(Color.ORANGE) ||
                color.equals(new Color(255, 165, 0))) {
                currentStateLabel.setForeground(Color.BLACK);
            } else {
                currentStateLabel.setForeground(Color.WHITE);
            }
        });
    }

    private void appendOutput(String text)
    {
        SwingUtilities.invokeLater(() -> {
            outputArea.append(text + "\n");
            outputArea.setCaretPosition(outputArea.getDocument().getLength());
        });
    }

    public void cleanup()
    {
        if (apiClient != null)
        {
            apiClient.disconnect();
        }
    }
}
