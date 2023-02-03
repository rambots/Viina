package com.rambots4571.chargedup.robot.utils;

import lombok.Getter;

public class LinkedList<E> {
  private Node<E> first;
  private Node<E> last;

  private int size = 0;

  @Getter
  public static class Node<E> {
    E item;
    Node<E> prev;
    Node<E> next;

    public Node(Node<E> prev, E item, Node<E> next) {
      this.item = item;
      this.prev = prev;
      this.next = next;
    }
  }

  public void linkFirst(E e) {
    final Node<E> f = first;
    final Node<E> newNode = new Node<>(null, e, f);
    first = newNode;
    if (f == null) last = newNode;
    else f.prev = newNode;
    size++;
  }

  public void linkLast(E e) {
    final Node<E> l = last;
    final Node<E> newNode = new Node<>(l, e, null);
    last = newNode;
    if (l == null) first = newNode;
    else l.next = newNode;
    size++;
  }

  public void add(E e) {
    linkLast(e);
  }

  /** Inserts element e before non-null Node succ. */
  void linkBefore(E e, Node<E> succ) {
    // assert succ != null;
    final Node<E> pred = succ.prev;
    final Node<E> newNode = new Node<>(pred, e, succ);
    succ.prev = newNode;
    if (pred == null) first = newNode;
    else pred.next = newNode;
    size++;
  }

  /** Unlinks non-null first node f. */
  public E unlinkFirst(Node<E> f) {
    // assert f == first && f != null;
    final E element = f.item;
    final Node<E> next = f.next;
    f.item = null;
    f.next = null; // help GC
    first = next;
    if (next == null) last = null;
    else next.prev = null;
    size--;
    return element;
  }

  /** Unlinks non-null last node l. */
  public E unlinkLast(Node<E> l) {
    // assert l == last && l != null;
    final E element = l.item;
    final Node<E> prev = l.prev;
    l.item = null;
    l.prev = null; // help GC
    last = prev;
    if (prev == null) first = null;
    else prev.next = null;
    size--;
    return element;
  }

  /** Unlinks non-null node x. */
  public E unlink(Node<E> x) {
    // assert x != null;
    final E element = x.item;
    final Node<E> next = x.next;
    final Node<E> prev = x.prev;

    if (prev == null) {
      first = next;
    } else {
      prev.next = next;
      x.prev = null;
    }

    if (next == null) {
      last = prev;
    } else {
      next.prev = prev;
      x.next = null;
    }

    x.item = null;
    size--;
    return element;
  }

  public Node<E> getFirst() {
    return first;
  }

  public Node<E> getLast() {
    return last;
  }

  public int size() {
    return size;
  }

  public int indexOf(Object o) {
    int index = 0;
    if (o == null) {
      for (Node<E> x = first; x != null; x = x.next) {
        if (x.item == null) return index;
        index++;
      }
    } else {
      for (Node<E> x = first; x != null; x = x.next) {
        if (o.equals(x.item)) return index;
        index++;
      }
    }
    return -1;
  }

  /** Returns the (non-null) Node at the specified element index. */
  public Node<E> node(int index) {
    // assert isElementIndex(index);

    if (index < (size >> 1)) {
      Node<E> x = first;
      for (int i = 0; i < index; i++) x = x.next;
      return x;
    } else {
      Node<E> x = last;
      for (int i = size - 1; i > index; i--) x = x.prev;
      return x;
    }
  }
}
