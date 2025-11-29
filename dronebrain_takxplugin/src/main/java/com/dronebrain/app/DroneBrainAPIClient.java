package com.rivet.app;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.invoke.MethodHandles;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.util.concurrent.CompletableFuture;

public class RivetAPIClient
{
    private static final Logger logger = LoggerFactory.getLogger(MethodHandles.lookup().lookupClass());

    private final HttpClient httpClient;
    private volatile String serverUrl = null;

    public RivetAPIClient()
    {
        this.httpClient = HttpClient.newBuilder()
                .connectTimeout(Duration.ofSeconds(10))
                .followRedirects(HttpClient.Redirect.NORMAL)
                .build();
    }

    public CompletableFuture<String> execute()
    {
        if (serverUrl == null || serverUrl.isEmpty())
        {
            return CompletableFuture.failedFuture(new IllegalStateException("Not connected to server"));
        }

        return sendRequest("/execute");
    }

    public CompletableFuture<String> stop()
    {
        if (serverUrl == null || serverUrl.isEmpty())
        {
            return CompletableFuture.failedFuture(new IllegalStateException("Not connected to server"));
        }

        return sendRequest("/stop");
    }

    public CompletableFuture<String> kill()
    {
        if (serverUrl == null || serverUrl.isEmpty())
        {
            return CompletableFuture.failedFuture(new IllegalStateException("Not connected to server"));
        }

        return sendRequest("/kill");
    }

    private CompletableFuture<String> sendRequest(String endpoint)
    {
        String fullUrl = serverUrl + endpoint;
        logger.info("Sending request to: {}", fullUrl);

        HttpRequest request = HttpRequest.newBuilder()
                .uri(URI.create(fullUrl))
                .POST(HttpRequest.BodyPublishers.noBody())
                .timeout(Duration.ofSeconds(30))
                .build();

        return httpClient.sendAsync(request, HttpResponse.BodyHandlers.ofString())
                .handle((response, throwable) -> {
                    if (throwable != null)
                    {
                        if (throwable instanceof java.net.ConnectException)
                        {
                            throw new RuntimeException("Cannot connect to server at " + serverUrl, throwable);
                        }
                        else if (throwable instanceof java.net.http.HttpTimeoutException)
                        {
                            throw new RuntimeException("Request timed out after 30 seconds", throwable);
                        }
                        else if (throwable.getCause() instanceof java.net.UnknownHostException)
                        {
                            throw new RuntimeException("Unknown host: " + serverUrl, throwable);
                        }
                        else
                        {
                            throw new RuntimeException("Network error: " + throwable.getMessage(), throwable);
                        }
                    }
                    return response;
                })
                .thenApply(response -> {
                    if (response.statusCode() >= 200 && response.statusCode() < 300)
                    {
                        String responseBody = response.body();
                        logger.info("Request successful: {}", responseBody);
                        return responseBody;
                    }
                    else
                    {
                        String errorMessage = "HTTP error: " + response.statusCode();
                        String responseBody = response.body();

                        if (responseBody != null && !responseBody.isEmpty())
                        {
                            errorMessage += " - " + responseBody;
                        }

                        logger.error("Request failed: {}", errorMessage);
                        throw new RuntimeException(errorMessage);
                    }
                });
    }

    public boolean isConnected()
    {
        return serverUrl != null && !serverUrl.isEmpty();
    }

    public CompletableFuture<Boolean> testConnection(String ipPort)
    {
        if (ipPort == null || ipPort.trim().isEmpty())
        {
            return CompletableFuture.failedFuture(new IllegalArgumentException("IP:Port cannot be null or empty"));
        }

        String url = ipPort.trim();

        // Ensure the URL has a proper protocol
        if (!url.startsWith("http://") && !url.startsWith("https://"))
        {
            url = "http://" + url;
        }

        // Remove trailing slash if present
        if (url.endsWith("/"))
        {
            url = url.substring(0, url.length() - 1);
        }

        String healthUrl = url + "/health";
        logger.info("Testing connection to: {}", healthUrl);

        HttpRequest request = HttpRequest.newBuilder()
                .uri(URI.create(healthUrl))
                .GET()
                .timeout(Duration.ofSeconds(5))
                .build();

        return httpClient.sendAsync(request, HttpResponse.BodyHandlers.ofString())
                .thenApply(response -> {
                    boolean success = response.statusCode() >= 200 && response.statusCode() < 300;
                    if (success)
                    {
                        logger.info("Connection test successful: {}", response.body());
                    }
                    else
                    {
                        logger.warn("Connection test returned HTTP {}", response.statusCode());
                    }
                    return success;
                })
                .exceptionally(throwable -> {
                    logger.warn("Connection test failed: {}", throwable.getMessage());
                    return false;
                });
    }

    public void connect(String ipPort)
    {
        if (ipPort == null || ipPort.trim().isEmpty())
        {
            throw new IllegalArgumentException("IP:Port cannot be null or empty");
        }

        String url = ipPort.trim();

        // Ensure the URL has a proper protocol
        if (!url.startsWith("http://") && !url.startsWith("https://"))
        {
            url = "http://" + url;
        }

        // Remove trailing slash if present
        if (url.endsWith("/"))
        {
            url = url.substring(0, url.length() - 1);
        }

        this.serverUrl = url;
        logger.info("Connected to: {}", this.serverUrl);
    }

    public void disconnect()
    {
        serverUrl = null;
        logger.info("Disconnected from server");
    }

    public String getServerUrl()
    {
        return serverUrl;
    }
}
