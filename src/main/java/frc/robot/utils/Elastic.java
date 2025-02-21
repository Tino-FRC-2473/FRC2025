// Copyright (c) 2023-2025 Gold87 and other Elastic contributors
// This software can be modified and/or shared under the terms
// defined by the Elastic license:
// https://github.com/Gold872/elastic-dashboard/blob/main/LICENSE

package frc.robot.utils;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;

public final class Elastic {
	private static final StringTopic NOTIF_TOPIC =
		NetworkTableInstance.getDefault().getStringTopic("/Elastic/RobotNotifications");
	private static final StringPublisher NOTIF_PUBLISHER =
		NOTIF_TOPIC.publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
	private static final StringTopic SELECTED_TAB_TOPIC =
		NetworkTableInstance.getDefault().getStringTopic("/Elastic/SelectedTab");
	private static final StringPublisher SELECTED_TAB_PUBLISHER =
		SELECTED_TAB_TOPIC.publish(PubSubOption.keepDuplicates(true));
	private static final ObjectMapper OBJECT_MAPPER = new ObjectMapper();

	/* ===================== Constants =========================== */
	private static final int DEFAULT_NOTIF_DISPLAY_TIME_MS = 3000;
	private static final int DEFAULT_NOTIF_WIDTH = 350;
	private static final int DEFAULT_NOTIF_HEIGTH = -1;
	private static final int SECS_TO_MS = 1000;

	/**
   * Sends an notification to the Elastic dashboard.
   * The notification is serialized as a JSON string
   * before being published.
   *
   * @param notification the {@link Notification} object containing notification details
   */
	public static void sendNotification(Notification notification) {
		try {
			NOTIF_PUBLISHER.set(OBJECT_MAPPER.writeValueAsString(notification));
		} catch (JsonProcessingException e) {
			e.printStackTrace();
		}
	}

	/**
   * Selects the tab of the dashboard with the given name.
   * If no tab matches the name, this will
   * have no effect on the widgets or tabs in view.
   *
   * <p>If the given name is a number, Elastic will select the tab whose index equals the number
   * provided.
   *
   * @param tabName the name of the tab to select
   */
	public static void selectTab(String tabName) {
		SELECTED_TAB_PUBLISHER.set(tabName);
	}

	/**
   * Selects the tab of the dashboard at the given index. If this index is greater than or equal to
   * the number of tabs, this will have no effect.
   *
   * @param tabIndex the index of the tab to select.
   */
	public static void selectTab(int tabIndex) {
		selectTab(Integer.toString(tabIndex));
	}

	/**
   * Represents an notification object to be sent to the Elastic dashboard. This object holds
   * properties such as level, title, description, display time, and dimensions to control how the
   * notification is displayed on the dashboard.
   */
	public static class Notification {
		@JsonProperty("level")
		private NotificationLevel level;

		@JsonProperty("title")
		private String title;

		@JsonProperty("description")
		private String description;

		@JsonProperty("displayTime")
		private int displayTimeMillis;

		@JsonProperty("width")
		private double width;

		@JsonProperty("height")
		private double height;

	/**
	 * Creates a new Notification with all default parameters. This constructor is intended to be
	 * used with the chainable decorator methods
	 *
	 * <p>Title and description fields are empty.
	 */
		public Notification() {
			this(NotificationLevel.INFO, "", "");
		}

	/**
	 * Creates a new Notification with all properties specified.
	 *
	 * @param notifLvl the level of the notification (e.g., INFO, WARNING, ERROR)
	 * @param notifTitle the title text of the notification
	 * @param notifDescription the descriptive text of the notification
	 * @param notifDisplayTimeMillis the time in ms for which the notification is displayed
	 * @param notifWidth the width of the notification display area
	 * @param notifHeight the height of the notification display area, inferred if below zero
	 */
		public Notification(
			NotificationLevel notifLvl,
			String notifTitle,
			String notifDescription,
			int notifDisplayTimeMillis,
			double notifWidth,
			double notifHeight) {
			this.level = notifLvl;
			this.title = notifTitle;
			this.displayTimeMillis = notifDisplayTimeMillis;
			this.description = notifDescription;
			this.height = notifHeight;
			this.width = notifWidth;
		}

	/**
	 * Creates a new Notification with default display time and dimensions.
	 *
	 * @param notifLvl the level of the notification
	 * @param notifTitle the title text of the notification
	 * @param notifDescription the descriptive text of the notification
	 */
		public Notification(
			NotificationLevel notifLvl,
			String notifTitle,
			String notifDescription) {
			this(notifLvl, notifTitle, notifDescription, DEFAULT_NOTIF_DISPLAY_TIME_MS,
				DEFAULT_NOTIF_WIDTH, DEFAULT_NOTIF_HEIGTH);
		}

	/**
	 * Creates a new Notification with a specified display time and default dimensions.
	 *
	 * @param notifLvl the level of the notification
	 * @param notifTitle the title text of the notification
	 * @param notifDescription the descriptive text of the notification
	 * @param notifDisplayTimeMillis the display time in milliseconds
	 */
		public Notification(
			NotificationLevel notifLvl, String notifTitle, String notifDescription,
				int notifDisplayTimeMillis) {
			this(notifLvl, notifTitle, notifDescription, notifDisplayTimeMillis,
				DEFAULT_NOTIF_WIDTH, DEFAULT_NOTIF_HEIGTH);
		}

	/**
	 * Creates a new Notification with specified dimensions and default display time. If the height
	 * is below zero, it is automatically inferred based on screen size.
	 *
	 * @param notifLvl the level of the notification
	 * @param notifTitle the title text of the notification
	 * @param notifDescription the descriptive text of the notification
	 * @param notifWidth the width of the notification display area
	 * @param notifHeight the height of the notification display area, inferred if below zero
	 */
		public Notification(
			NotificationLevel notifLvl, String notifTitle, String notifDescription,
				double notifWidth, double notifHeight) {
			this(notifLvl, notifTitle, notifDescription, DEFAULT_NOTIF_DISPLAY_TIME_MS, notifWidth,
				notifHeight);
		}

		/**
		 * Updates the level of this notification.
		 *
		 * @param notifLvl the level to set the notification to
		 */
		public void setLevel(NotificationLevel notifLvl) {
			this.level = notifLvl;
		}

		/**
		 * @return the level of this notification
		 */
		public NotificationLevel getLevel() {
			return level;
		}

		/**
		 * Updates the title of this notification.
		 *
		 * @param notifTitle the title to set the notification to
		 */
		public void setTitle(String notifTitle) {
			this.title = notifTitle;
		}

		/**
		 * Gets the title of this notification.
		 *
		 * @return the title of this notification
		 */
		public String getTitle() {
			return title;
		}

		/**
		 * Updates the description of this notification.
		 *
		 * @param notifDescription the description to set the notification to
		 */
		public void setDescription(String notifDescription) {
			this.description = notifDescription;
		}

		/**
		 * Gets the description of this notification.
		 *
		 * @return the description of this notification
		 */
		public String getDescription() {
			return description;
		}

		/**
		 * Updates the display time of the notification.
		 *
		 * @param seconds the number of seconds to display the notification for
		 */
		public void setDisplayTimeSeconds(double seconds) {
			setDisplayTimeMillis((int) Math.round(seconds * SECS_TO_MS));
		}

		/**
		 * Updates the display time of the notification in milliseconds.
		 *
		 * @param notifDisplayTimeMillis the number of milliseconds to display the notification for
		 */
		public void setDisplayTimeMillis(int notifDisplayTimeMillis) {
			this.displayTimeMillis = notifDisplayTimeMillis;
		}

		/**
		 * Gets the display time of the notification in milliseconds.
		 *
		 * @return the number of milliseconds the notification is displayed for
		 */
		public int getDisplayTimeMillis() {
			return displayTimeMillis;
		}

		/**
		 * Updates the width of the notification.
		 *
		 * @param notifWidth the width to set the notification to
		 */
		public void setWidth(double notifWidth) {
			this.width = notifWidth;
		}

		/**
		 * Gets the width of the notification.
		 *
		 * @return the width of the notification
		 */
		public double getWidth() {
			return width;
		}

		/**
		 * Updates the height of the notification.
		 *
		 * <p>If the height is set to -1, the height will be determined
		 * <p>automatically by the dashboard
		 *
		 * @param notifHeight the height to set the notification to
		 */
		public void setHeight(double notifHeight) {
			this.height = notifHeight;
		}

		/**
		 * Gets the height of the notification.
		 *
		 * @return the height of the notification
		 */
		public double getHeight() {
			return height;
		}

		/**
		 * Modifies the notification's level and returns itself to allow for method chaining.
		 *
		 * @param notifLvl the level to set the notification to
		 * @return the current notification
		 */
		public Notification withLevel(NotificationLevel notifLvl) {
			this.level = notifLvl;
			return this;
		}

		/**
		 * Modifies the notification's title and returns itself to allow for method chaining.
		 *
		 * @param notifTitle the title to set the notification to
		 * @return the current notification
		 */
		public Notification withTitle(String notifTitle) {
			setTitle(notifTitle);
			return this;
		}

		/**
		 * Modifies the notification's description and returns itself to allow for method chaining.
		 *
		 * @param notifDescription the description to set the notification to
		 * @return the current notification
		 */
		public Notification withDescription(String notifDescription) {
			setDescription(notifDescription);
			return this;
		}

		/**
		 * Modifies the notification's display time and returns itself to allow for method chaining.
		 *
		 * @param seconds the number of seconds to display the notification for
		 * @return the current notification
		 */
		public Notification withDisplaySeconds(double seconds) {
			return withDisplayMilliseconds((int) Math.round(seconds * SECS_TO_MS));
		}

		/**
		 * Modifies the notification's display time and returns itself to allow for method chaining.
		 *
		 * @param notifDisplayTimeMillis the number of milliseconds to display the notification for
		 * @return the current notification
		 */
		public Notification withDisplayMilliseconds(int notifDisplayTimeMillis) {
			setDisplayTimeMillis(notifDisplayTimeMillis);
			return this;
		}

		/**
		 * Modifies the notification's width and returns itself to allow for method chaining.
		 *
		 * @param notifWidth the width to set the notification to
		 * @return the current notification
		 */
		public Notification withWidth(double notifWidth) {
			setWidth(notifWidth);
			return this;
		}

		/**
		 * Modifies the notification's height and returns itself to allow for method chaining.
		 *
		 * @param notifHeight the height to set the notification to
		 * @return the current notification
		 */
		public Notification withHeight(double notifHeight) {
			setHeight(notifHeight);
			return this;
		}

		/**
		 * Modifies the notification's height and returns itself to allow for method chaining.
		 *
		 * <p>This will set the height to -1 to have it automatically determined by the dashboard
		 *
		 * @return the current notification
		 */
		public Notification withAutomaticHeight() {
			setHeight(-1);
			return this;
		}

		/**
		 * Modifies the notification to disable the auto dismiss behavior.
		 *
		 * <p>This sets the display time to 0 milliseconds
		 *
		 * <p>The auto dismiss behavior can be re-enabled by setting the display time to a number
		 * greater than 0
		 *
		 * @return the current notification
		 */
		public Notification withNoAutoDismiss() {
			setDisplayTimeMillis(0);
			return this;
		}

	/**
	 * Represents the possible levels of notifications for the Elastic dashboard. These levels are
	 * used to indicate the severity or type of notification.
	 */
		public enum NotificationLevel {
		/** Informational Message. */
		INFO,
		/** Warning message. */
		WARNING,
		/** Error message .*/
		ERROR
		}
	}
}
