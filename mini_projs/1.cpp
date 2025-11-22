#include <iostream>
#include <cstdlib>

template <typename T>
struct Node 
{
    Node<T>* next;
    Node<T>* prev;
    T data;
    
    Node(const T& data_val) : data(data_val), next(nullptr), prev(nullptr) {}
};

template <typename T>
class List
{
public:
    List() {}

    virtual void append(const T& data_val) = 0;
    virtual void deleteNode(const T& data_val) = 0;
    virtual void insertAfter(const T& target, const T& data_val) = 0;
    virtual void display() = 0;

    virtual ~List() {}
};

template <typename T>
class SingleLinkedList : public List<T>
{
public:
    void append(const T& data_val) override
    {
        Node<T>* newNode = createNode(data_val);
        if (!head) {
            head = newNode;
            return;
        }
        Node<T>* temp = head;
        while (temp->next) temp = temp->next;
        temp->next = newNode; 
    }

    void deleteNode(const T& data_val) override 
    {
        if (!head) {
            std::cout << "Empty List!" << std::endl;
            return;
        }

        if (head->data == data_val) {
            Node<T>* ToDelete = head;
            if (head->next) head = head->next;
            ToDelete->~Node<T>();
            free(ToDelete);
            return;
        }

        Node<T>* temp = head;
        while (temp->next && temp->next->next) {
            if (temp->next->data == data_val) break;
            temp = temp->next;
        }

        if (!temp->next) {
            std::cout << "Value not found" << std::endl;
        }

        Node<T>* ToDelete = temp->next;
        temp->next = ToDelete->next;
        ToDelete->~Node<T>();
        free(ToDelete);
    }

    void insertAfter(const T& target, const T& data_val) override
    {
        Node<T>* newNode = createNode(data_val);
        if (!head) return;
        if (head->data == target) {
            newNode->next = head->next;
            head->next = newNode;
            return;
        }
        Node<T>* temp = head;
        while (temp->next) {
            if (temp->data == target) break;
            temp = temp->next;
        }

        if (temp->data != target) {
            std::cout << "Target Not Found" << std::endl;
            return;
        }

        newNode->next = temp->next;
        temp->next = newNode;
        return;
    }

    void display() override
    {
        if (!head) {
            std::cout << "Empty list" << std::endl;
            return;
        }

        Node<T>* temp = head;
        while (temp->next) {
            std::cout << temp->data << " -> ";
            temp = temp->next;
        }
        std::cout << temp->data << " -> NULL" << std::endl;
        return;
    }
    
    ~SingleLinkedList() {
        Node<T>* curr = head;
        while (curr) {
            Node<T>* next = curr->next;
            curr->~Node<T>();
            free(curr);
            curr = next;
        }
    }
private:
    Node<T>* head{nullptr};
    Node<T>* createNode(const T& data_val) 
    {
        void* mem = malloc(sizeof(Node<T>));
        Node<T>* newNode = new(mem) Node<T>(data_val);
        return newNode;
    }
};

template <typename T>
class DoubleLinkedList : public List<T>
{
public:
    void append(const T& data_val) override
    {
        Node<T>* newNode = createNode(data_val);
        if (!head) head = newNode;

        Node<T>* tail = head->prev;
        tail->next = newNode;
        newNode->next = head;
        newNode->prev = tail;
        head->prev = newNode;
        return;
    }

    void deleteNode(const T& data_val) override
    {
        if (!head) {
            std::cout << "Empty List" << std::endl;
            return;
        }

        if (head->data == data_val) {
            if (!head->next) {
                head->~Node<T>();
                free(head);
                return;
            }
            Node<T>* ToDelete = head;
            Node<T>* newHead = head->next;
            ToDelete->prev->next = ToDelete->next;
            ToDelete->next->prev = ToDelete->prev;
            ToDelete->~Node<T>();
            free(ToDelete);
            head = newHead;
            return; 
        }

        Node<T>* temp = head;
        while (temp->next != head) {
            if (temp->data == data_val) break;
        }
        if (temp->next == head) {
            std::cout << "Value not found" << std::endl;
            return;
        }
        Node<T>* ToDelete = temp;
        ToDelete->prev->next = ToDelete->next;
        ToDelete->next->prev = ToDelete->prev;
        ToDelete->~Node<T>();
        free(ToDelete);
        return;
    }

    void insertAfter(const T& target, const T& data_val)
    {
        Node<T>* newNode = createNode(data_val);
        if (!head) {
            std::cout << "Empty list" << std::endl;
            return;
        }

        if (head->data == target) {
            if (!head->next) {
                head->next = newNode;
                head->prev = newNode;
                newNode->prev = head;
                newNode->next = head;
                return;
            }
        }

        Node<T>* temp = head;
        while (temp->next != head) {
            if (temp->data == target) break;
            temp = temp->next;
        }
        
        if (temp->next == head && temp->data != target) {
            std::cout << "Target not found" << std::endl;
            return;
        }
        newNode->next = temp->next;
        newNode->prev = temp;
        temp->next = newNode;
        newNode->next->prev = newNode;
        return;
    }

    void display() override
    {
        if (!head) {
            std::cout << "Empty list" << std::endl;
            return;
        }
        Node<T>* temp = head;
        while (temp->next != head) {
            std::cout << temp->data << " <-> ";
            temp = temp->next;
        }
        std::cout << temp->data << " <-> Head" << std::endl;
        return;
    }
    ~DoubleLinkedList() {
        Node<T>* curr = head;
        while (curr->next != head) {
            Node<T>* next = curr->next;
            curr->~Node<T>();
            free(curr);
            curr = next;
        }
    }
private:
    Node<T>* head{nullptr};
    Node<T>* createNode(const T& data_val)
    {
        void* mem = malloc(sizeof(Node<T>));
        Node<T>* newNode = new(mem) Node<T>(data_val);
        newNode->next = newNode;
        newNode->prev = newNode;
        return newNode;
    }
};

int main() 
{
    List<double>* list_ptr = new SingleLinkedList<double>();
    list_ptr->append(1);
    list_ptr->append(1.2);
    list_ptr->append(1.3);
    list_ptr->append(1.45);
    list_ptr->append(5);
    list_ptr->insertAfter(1.45, 3.3);
    list_ptr->display();

    list_ptr = new DoubleLinkedList<double>();
    for (double i{1}; i < 8; i = i + 0.5) {
        list_ptr->append(i);
    }
    list_ptr->display();
    for (double i{1}; i < 5; i = i + 0.5) {
        list_ptr->deleteNode(i);
    }
    list_ptr->display();
    return 0;
}