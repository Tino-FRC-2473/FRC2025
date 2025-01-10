# For testing and diagnosing connectivity issues across parts on the robot if needed
import os
import time
import csv
def main():
    try:
        with open(f"pinglogs {time.ctime(time.time())}.csv", mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Timestamp", "Laptop", "Router", "Rio"])
            # You may need to edit the laptop IP
            ips = ["10.24.73.211", "10.24.73.1", "10.24.73.2"]
            while True:
                replies = [time.ctime(time.time())]
                for ip in ips:
                    reply = os.system(f"ping -c 1 {ip} > /dev/null 2>&1")
                    replies.append("SUCCESS (0)" if reply == 0 else f"FAIL ({reply})")
                writer.writerow(replies)
                csvfile.flush()
                print(replies)
                time.sleep(0.5)
    except KeyboardInterrupt:
        print("Ping logging stopped.")
        writer.close()
    except Exception as e:
        writer.close()
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()