package com.rivet.app;

import gov.takx.api.annotation.InvokeOnEDT;
import gov.takx.api.plugin.annotations.AppPlugin;
import gov.takx.api.plugin.annotations.PluginDescriptor;
import gov.takx.api.plugin.annotations.PluginSpecification;
import gov.takx.api.plugin.app.AAppPlugin;
import gov.takx.api.ui.ComponentPosition;
import gov.takx.api.ui.IUISpaceManager;
import gov.takx.platform.ui.ComponentDescriptor;
import gov.takx.platform.ui.ComponentDescriptorBuilder;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.invoke.MethodHandles;

@AppPlugin(
        spec = @PluginSpecification(
                family = RivetAppConstants.FAMILY,
                type = RivetAppConstants.TYPE,
                version = RivetAppConstants.VERSION,
                iconPath = "rivet.png"
        ),
        descriptor = @PluginDescriptor(
                name = "Rivet",
                description = "Aircraft control plugin for execute, stop, and kill commands",
                longDescription = "Rivet plugin provides remote command execution capabilities for aircraft systems"
        ),
        canRunHeadless = false
)
@SuppressWarnings("UnusedDeclaration")
public class RivetAppPlugin extends AAppPlugin
{
    private static final Logger logger = LoggerFactory.getLogger(MethodHandles.lookup().lookupClass());

    private RivetMainPanel mainPanel;

    @Override
    public void start()
    {
        setupUI();
        logger.info("Rivet plugin started");
    }

    @InvokeOnEDT
    private void setupUI()
    {
        if (delegate == null)
        {
            logger.error("Plugin delegate is null, cannot setup UI");
            return;
        }

        mainPanel = new RivetMainPanel(delegate);

        ComponentDescriptor componentDescriptor = new ComponentDescriptorBuilder(mainPanel, "Rivet")
                .setIcon(getClass().getClassLoader().getResource("rivet.png"))
                .setComponentPosition(ComponentPosition.RIGHT)
                .build();

        IUISpaceManager uiSpaceManager = delegate.getUISpaceManager();
        uiSpaceManager.addComponent(componentDescriptor, mainPanel);
        uiSpaceManager.showComponent(mainPanel);
    }

    @Override
    public void stop()
    {
        teardownUI();
        logger.info("Rivet plugin stopped");
    }

    @InvokeOnEDT
    private void teardownUI()
    {
        if (mainPanel != null)
        {
            if (delegate != null && delegate.getUISpaceManager() != null)
            {
                delegate.getUISpaceManager().removeComponent(mainPanel);
            }
            mainPanel.cleanup();
            mainPanel = null;
        }
    }

    @Override
    public void shutdown()
    {
        mainPanel = null;
    }
}
