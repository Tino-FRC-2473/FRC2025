# For testing and diagnosing connectivity issues across parts on the robot if needed
import os
import time
import csv
import subprocess

def ping(ip):
    try:
        subprocess.check_call(["ping", "-c", "1", ip], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return "SUCCESS (0)"
    except subprocess.CalledProcessError as e:
        return f"FAIL ({e.returncode})"
    except Exception as e:
        return f"ERROR ({e})"

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
                    replies.append(ping(ip))
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