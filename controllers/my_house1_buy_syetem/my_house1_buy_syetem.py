"""my_house1_buy_syetem controller."""

from controller import Robot, Emitter, Receiver
import json
import random
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

house1_emitter = robot.getDevice("house_emitter1")
house1_receiver = robot.getDevice("house_receiver1")
house1_receiver.enable(timestep)

order_counter = 1000
pending_orders = {}
send_interval = 2.0
last_send_time = -1.0

print("The initialization of the house order system has been completed\n")

House_List = ["Loading Bay A",
              "Loading Bay B",
              "Loading Bay C",
              "Loading Bay D",]

def generate_order():
    global order_counter
    order_data = {
        "type" : "order",
        "order_id" : f"O-{order_counter}",
        "timestamp" : robot.getTime(),
        "status" : "pending",
        "destination" : random.choice(House_List),
        "priority" : "normal",
    }
    order_counter += 1
    return order_data

# Send order
def send_order(order_data):
    payload = json.dumps(order_data).encode('utf-8')
    house1_emitter.send(payload)
    pending_orders[order_data["order_id"]] = order_data["timestamp"]
    print(f"[HOUSE] The order has been sent: {order_data['order_id']} -> {order_data.get('destination')}\n")

# Handle input
def handle_incoming():
    while house1_receiver.getQueueLength() > 0:
        raw = house1_receiver.getString()
        house1_receiver.nextPacket()
        try:
            msg = json.loads(raw)
        except Exception:
            print(f"[HOUSE] Non-json data was received and has been ignored\n")
            continue
        if msg['order_status'] == 'Sending':
            print(f"[HOUSE] The order is in transit\n")
            state_value = 1
            return state_value
        elif msg['order_status'] == 'Complete':
            print(f"[HOUSE] The order has been delivered\n")
            print(f"[HOUSE] The order address: {msg['destination']}\n")
            print(f"[HOUSE] The delivery time: {msg['timestamp']}\n")
            state_value = 1
            return state_value
        elif msg['order_status'] == 'Home':
            state_value = 0
            return state_value

# Enter here exit cleanup code.
if __name__ == "__main__":
    state = 0
    while robot.step(timestep) != -1:
        recv_state = handle_incoming()

        if recv_state == 0:
            state = 0

        if state == 0:
            order = generate_order()
            send_order(order)
            state = 1

        elif state == 1:
            state = recv_state if recv_state is not None else 1
                 