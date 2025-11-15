#include <iostream>
using namespace std;

template <typename T>
struct Node {
    Node<T>* next;
    Node<T>* prev;
    T data;                                          
};

template <typename T>
class LL {
private:
    Node<T>* create_node(const T data_val) {
        Node<T>* newNode = new Node<T>;
        newNode->data = data_val;
        newNode->next = newNode;
        newNode->prev = newNode;
        return newNode;
    }
public:
    Node<T>* head;
    LL(const T val) {
        head = create_node(val);
    }
    ~LL() {clear();}

    void clear() {
        if (!head) return;
        Node<T>* curr = head;
        while (curr != head) {
            curr = curr->next;
            delete curr;
        }
        delete head;
        head = nullptr;
        return;
    }

    void append(const T val) {
        Node<T>* newNode = create_node(val);
        if (!head) {
            head = newNode;
            return;
        }
        Node<T>* tail = head->prev;
        tail->next = newNode;
        head->prev = newNode;
        newNode->next = head;
        newNode->prev = tail;
        return;
    }

    void insert_beginning(const T val) {
        Node<T>* newHead = create_node(val);
        if (!head) {
            head = newHead;
            return;
        }
        Node<T>* tail = head->prev;
        Node<T>* prevHead = head;

        prevHead->prev = newHead;
        newHead->next = head;
        newHead->prev = tail;
        tail->next = newHead;
        head = newHead;
        return;
    }

    void deleteNode(const T val){
        if (!head) {
            cout << "Empty list!" << endl;
            return;
        }
        if (head->data == val) {
            Node<T>* tail = head->prev;
            Node<T>* newHead = head->next;
            tail->next = newHead;
            newHead->prev = tail;
            delete head;
            head = newHead;
            }

        Node<T>* temp = head;
        while (temp->next != head && temp->data != val) { 
            temp = temp->next;
            if (temp->data == val) {
                cout << "Element not found" << endl;
                return;
            }
        }
        temp->next->prev = temp->prev;
        temp->prev->next = temp->next;
        delete temp;
        return;
    }

    void displayFromHead() {
        if (!head) {
            cout << "Empty list" << endl;
            return;
        }
        if (head->next == head) {
            cout << head->data << " <-> head" << endl;
            return;
        }
        Node<T>* temp = head;
        cout << "head <-> ";
        while (temp->next != head) {
            cout << temp->data << " <-> ";
            temp = temp->next;
        }
        cout << temp->data << " <-> head" << endl;
        return;
    }

    void displayFromTail() {
        Node<T>* tail = head->prev;
        if (!head) {
            cout << "Empty list" << endl;
            return;
        }

        if (tail == head) {
            cout <<  "head <->" << head->data << " <-> head" << endl;
            return;
        }

        cout << "head <-> ";
        while (tail->prev != head) {
            cout << tail->data << " <-> ";
            tail = tail->prev;
        }
        cout << tail->data << " <-> ";
        cout << tail->prev->data << " <-> head" << endl;
        return;
    }
};

int main() {
    LL<int> list(5);
    for (size_t i{1}; i <= 10; ++i) {
        list.append(i);
    }
    list.insert_beginning(0);
    list.deleteNode(0);
    list.displayFromHead();
}